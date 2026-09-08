"""wdt 的行为测试台：喂对了不叫、不喂会叫、喂早了也算故障。

窗口看门狗的意义全在最后一条：喂早跟喂晚一样是程序跑飞的征兆。

两处结构上的讲究：
  · 故障标志用 CReg。测试序列要在中途把它清掉重来，而 pins 规则每拍都可能
    置位；用普通寄存器两条规则抢同一个写口，序列规则会被判不够紧急而永不触发。
  · 检查点都要等几拍。计数器归零那一拍才写 fired，中断再下一拍才被采到。
"""
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)

(out / "WdtTb.bsv").write_text('''package WdtTb;

import RegIf::*;
import Wdt::*;

// 由 tb/mkwdttb.py 生成，勿手改。

Bit#(8) rCTRL = 8'h00;
Bit#(8) rLOAD = 8'h04;
Bit#(8) rCNT  = 8'h08;
Bit#(8) rWIN  = 8'h0C;
Bit#(8) rFEED = 8'h10;

Bit#(32) feedKey = 32'hA5A5_5A5A;

typedef enum { Setup, RunDown, FeedOk, AfterFeed, LetExpire, CheckFire,
               Recover, FeedEarly, CheckEarly, Done }
  Phase deriving (Bits, Eq);

(* synthesize *)
module mkWdtTb(Empty);
  WdtIfc#(8, 32, 16) w <- mkWdt(WdtCfg { window: True });

  Reg#(Phase)    ph  <- mkReg(Setup);
  Reg#(Bit#(8))  s   <- mkReg(0);
  Reg#(Bit#(32)) cyc <- mkReg(0);
  Reg#(Bool)     bad <- mkReg(False);
  // 口 0 归 pins 置位，口 1 归测试序列读取与清零，清零压过同拍的置位
  Reg#(Bool)     rstSeen[2] <- mkCReg(2, False);
  Reg#(Bool)     irqSeen[2] <- mkCReg(2, False);

  rule pins;
    if (w.pins.rst_out == 1) rstSeen[0] <= True;
    if (w.irq) irqSeen[0] <= True;
  endrule

  rule tick;
    cyc <= cyc + 1;
    if (cyc > 20000) begin
      $display("TIMEOUT in phase %0d", pack(ph));
      $finish(1);
    end
  endrule

  function Action wr(Bit#(8) a, Bit#(32) v) = action
    let _ <- w.regs.access(RegReq { addr: a, write: True,
                                    wdata: v, wstrb: 4'hF });
  endaction;

  // 装 40、窗口 20：计数降到 20 以下才准喂
  rule setup (ph == Setup);
    case (s)
      0: wr(rLOAD, 40);
      1: wr(rWIN, 20);
      // 使能的上升沿自己会装载计数器，这里再喂一口反而落在窗口之外
      2: wr(rCTRL, 32'h1);       // en，中断模式（rstmode = 0）
      default: ph <= RunDown;
    endcase
    s <= s + 1;
  endrule

  // 等它数到窗口以下
  rule runDown (ph == RunDown);
    let x <- w.regs.access(RegReq { addr: rCNT, write: False,
                                    wdata: 0, wstrb: 4'hF });
    if (x.rdata < 20 && x.rdata > 5) ph <= FeedOk;
  endrule

  rule feedOk (ph == FeedOk);
    wr(rFEED, feedKey);
    ph <= AfterFeed;
    s  <= 0;
  endrule

  // 窗口内喂过之后不该有任何故障
  rule afterFeed (ph == AfterFeed);
    if (s > 3) begin
      Bool wrong = False;
      if (irqSeen[1] || rstSeen[1]) begin
        $display("FAIL a feed inside the window still raised a fault");
        wrong = True;
      end
      if (wrong) bad <= True;
      ph <= LetExpire;
    end
    s <= s + 1;
  endrule

  // 不喂了，让它数到底
  rule letExpire (ph == LetExpire);
    let x <- w.regs.access(RegReq { addr: rCNT, write: False,
                                    wdata: 0, wstrb: 4'hF });
    if (x.rdata == 0) begin ph <= CheckFire; s <= 0; end
  endrule

  // 归零那一拍才写 fired，中断再下一拍才被采到，所以等几拍再看
  rule checkFire (ph == CheckFire);
    if (s > 3) begin
      Bool wrong = False;
      if (!irqSeen[1]) begin
        $display("FAIL expiry did not raise the interrupt");
        wrong = True;
      end
      if (rstSeen[1]) begin
        $display("FAIL reset asserted although rstmode is zero");
        wrong = True;
      end
      if (wrong) bad <= True;
      ph <= Recover;
      s  <= 0;
    end
    else s <= s + 1;
  endrule

  // 喂一口恢复，把故障标志清掉重新看
  rule recover (ph == Recover);
    if (s == 0) wr(rFEED, feedKey);
    if (s == 3) begin irqSeen[1] <= False; rstSeen[1] <= False; end
    if (s == 4) begin ph <= FeedEarly; s <= 0; end
    else s <= s + 1;
  endrule

  // 计数器刚装满不久，远在窗口之上——这一口是喂早了
  rule feedEarly (ph == FeedEarly);
    wr(rFEED, feedKey);
    ph <= CheckEarly;
    s  <= 0;
  endrule

  rule checkEarly (ph == CheckEarly);
    if (s > 3) begin
      if (!irqSeen[1]) begin
        $display("FAIL an early feed was accepted, no fault raised");
        bad <= True;
      end
      ph <= Done;
    end
    s <= s + 1;
  endrule

  rule fin (ph == Done);
    if (bad) $display("FAILED");
    else $display("PASS wdt: fed in window is quiet, expiry fires, early feed faults");
    $finish(bad ? 1 : 0);
  endrule
endmodule

endpackage
''', encoding="utf-8")
print("  wdt 行为测试台就位")
