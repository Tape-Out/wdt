"""wdt 的行为测试台：喂对了不叫、不喂会叫、喂早了算不算故障要看窗口开没开。

认矩阵：第二个参数是本次这一点的旋钮，包名与期望都照它改。窗口关掉的那一
点，最后一段的期望正好反过来——喂早不再是故障，而是一次正常的重装。

两处结构上的讲究：
  · 故障标志用 CReg。测试序列要在中途清掉重来，而 pins 规则每拍都可能置位；
    用普通寄存器两条规则抢同一个写口，序列规则会被判不够紧急而永不触发。
  · 检查点都要等几拍。计数器归零那一拍才写 fired，中断再下一拍才被采到。
"""
import json
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)
cfg = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
label = cfg.get("label", "")
knobs = cfg.get("knobs", {})
width = int(knobs.get("width", 32))
window = bool(knobs.get("window", True))

# 窗口开着：喂早等于故障。窗口关着：喂早只是提前重装，不该有任何动静。
if window:
    early_check = """      if (!irqSeen[1]) begin
        $display("FAIL an early feed was accepted, no fault raised");
        bad <= True;
      end"""
    verdict = ("fed in window is quiet, expiry fires, early feed faults, "
           "a wrong key does not feed, periodic reloads without one")
else:
    early_check = """      if (irqSeen[1]) begin
        $display("FAIL an early feed raised a fault although the window is off");
        bad <= True;
      end"""
    verdict = ("fed is quiet, expiry fires, an early feed is just a reload, "
           "a wrong key does not feed, periodic reloads without one")

TEMPLATE = '''package Wdt@L@Tb;

import RegIf::*;
import Wdt::*;

// 由 tb/mkwdttb.py 生成，勿手改。这一点：width=@W@ window=@WIN@

Bit#(8) rCTRL = 8'h00;
Bit#(8) rLOAD = 8'h04;
Bit#(8) rCNT  = 8'h08;
Bit#(8) rWIN  = 8'h0C;
Bit#(8) rFEED = 8'h10;

Bit#(32) feedKey = 32'hA5A5_5A5A;

typedef enum { Setup, RunDown, FeedOk, AfterFeed, LetExpire, CheckFire,
               Recover, FeedEarly, CheckEarly, BadKey, RunLow, FeedBad,
               CheckBadKey, Periodic, CheckPeriodic, Done }
  Phase deriving (Bits, Eq);

(* synthesize *)
module mkWdt@L@Tb(Empty);
  WdtIfc#(8, 32, @W@) w <- mkWdt(WdtCfg { window: @WIN@ });

  Reg#(Phase)    ph  <- mkReg(Setup);
  Reg#(Bit#(8))  s   <- mkReg(0);
  Reg#(Bit#(32)) cyc <- mkReg(0);
  Reg#(Bool)     bad <- mkReg(False);
  // 口 0 归 pins 置位，口 1 归测试序列读取与清零，清零压过同拍的置位
  Reg#(Bool)     rstSeen[2] <- mkCReg(2, False);
  Reg#(Bool)     irqSeen[2] <- mkCReg(2, False);
  Reg#(Bit#(32)) prev <- mkReg(0);
  Reg#(Bit#(32)) mark <- mkReg(0);

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
@EARLY@
      ph <= BadKey;
      s  <= 0;
    end
    else s <= s + 1;
  endrule

  // 口令写错必须无效。跑飞的程序随手写一个字过来就能把狗喂了，口令就白设了。
  rule badKey (ph == BadKey);
    case (s)
      0: wr(rCTRL, 32'h0);
      1: wr(rLOAD, 20);
      2: wr(rCTRL, 32'h1);
      default: noAction;
    endcase
    if (s > 3) begin ph <= RunLow; s <= 0; end
    else s <= s + 1;
  endrule

  rule runLow (ph == RunLow);
    let x <- w.regs.access(RegReq { addr: rCNT, write: False,
                                    wdata: 0, wstrb: 4'hF });
    if (x.rdata < 10) begin
      mark <= x.rdata;
      ph   <= FeedBad;
    end
  endrule

  rule feedBad (ph == FeedBad);
    wr(rFEED, 32'hDEAD_BEEF);
    ph <= CheckBadKey;
    s  <= 0;
  endrule

  rule checkBadKey (ph == CheckBadKey);
    let x <- w.regs.access(RegReq { addr: rCNT, write: False,
                                    wdata: 0, wstrb: 4'hF });
    if (s > 3) begin
      if (x.rdata > mark) begin
        $display("FAIL a wrong key still fed the watchdog");
        bad <= True;
      end
      ph <= Periodic;
      s  <= 0;
    end
    else s <= s + 1;
  endrule

  // 关掉再开，让使能的上升沿把新的装载值放进去
  rule periodic (ph == Periodic);
    case (s)
      0: wr(rCTRL, 32'h0);
      1: wr(rLOAD, 12);
      2: wr(rCTRL, 32'h5);            // en | periodic
      default: noAction;
    endcase
    // s 只许有一条写路径：case 里写一次、外面再写一次就是 G0004
    if (s > 3) begin ph <= CheckPeriodic; s <= 0; end
    else s <= s + 1;
  endrule

  // 判据不能看中断：不带周期模式时中断也一直举着，看中断分不出来。
  // 要看计数器——不喂狗而它自己涨回去，只可能是到期自动重装。
  rule checkPeriodic (ph == CheckPeriodic);
    let x <- w.regs.access(RegReq { addr: rCNT, write: False,
                                    wdata: 0, wstrb: 4'hF });
    Bool rose = s > 0 && x.rdata > prev;
    Bool over = s > 200;
    if (over && !rose) begin
      $display("FAIL the counter never reloaded: periodic mode is not working");
      bad <= True;
    end
    prev <= x.rdata;
    s    <= s + 1;
    ph   <= (rose || over) ? Done : CheckPeriodic;
  endrule

  rule fin (ph == Done);
    if (bad) $display("FAILED");
    else $display("PASS wdt: @VERDICT@");
    $finish(bad ? 1 : 0);
  endrule
endmodule

endpackage
'''

txt = (TEMPLATE.replace("@L@", label)
               .replace("@W@", str(width))
               .replace("@WIN@", "True" if window else "False")
               .replace("@EARLY@", early_check)
               .replace("@VERDICT@", verdict))
(out / f"Wdt{label}Tb.bsv").write_text(txt, encoding="utf-8")
print(f"  wdt 行为测试台就位：width={width} window={window}")
