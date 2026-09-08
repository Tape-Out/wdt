package Wdt;

import RegIf::*;
import WdtRegs::*;

// 本包不认识任何总线：对外只给中立的 RegIf，接哪种总线由 wrap 或装配决定。
typedef struct {
  Bool window;
} WdtCfg;

// 喂狗口令。写别的值一律无效——跑飞的程序随手写一个字过来的概率因此很低。
Bit#(32) feedKey = 32'hA5A5_5A5A;

interface WdtPins;
  (* always_ready, result = "rst_out" *) method Bit#(1) rst_out;
endinterface

interface WdtIfc#(numeric type aw, numeric type dw, numeric type width);
  interface RegIf#(aw, dw) regs;
  interface WdtPins        pins;
  (* always_ready *) method Bool irq;
endinterface

module mkWdt#(WdtCfg cfg)(WdtIfc#(aw, dw, width))
    provisos (Mul#(TDiv#(dw, 8), 8, dw), Add#(_a, 8, aw), Add#(_b, width, dw),
              Add#(_c, 32, dw), Add#(_d, 1, dw));

  WdtRegsIfc#(aw, dw, width) r <- mkWdtRegs(WdtRegsCfg { window: cfg.window });

  Reg#(Bit#(width)) cnt   <- mkReg(0);
  Reg#(Bool)        fired <- mkReg(False);
  Reg#(Bool)        fedPend <- mkReg(False);
  // 使能的上升沿要装载计数器。不装的话，使能那一拍计数器还是 0，
  // 当场就判过期——跟 timer 的比较值复位为 0 是同一类毛病。
  Reg#(Bit#(1))     enPrev  <- mkReg(0);

  // swmod 的脉冲与寄存器的新值差一拍：脉冲在写的当拍发出，寄存器下一拍才有新值。
  // 所以先记下脉冲，下一拍再读口令。
  rule mark;
    fedPend <= r.feed_key_wr;
  endrule

  rule tick;
    r.cnt_in(cnt);
    enPrev <= r.ctrl_en;
    Bool armed = r.ctrl_en == 1 && enPrev == 0;
    Bool fed   = armed || (fedPend && r.feed_key == feedKey);
    // 开了窗口就不许喂早：喂早跟喂晚一样算故障，这正是窗口看门狗的用处
    // 刚使能的那一次装载不算「喂早」
    Bool early = cfg.window && fed && !armed && cnt > r.win;
    if (fed && !early) begin
      cnt   <= r.load;
      fired <= False;
    end else if (r.ctrl_en == 1) begin
      if (cnt == 0) fired <= True;
      else cnt <= cnt - 1;
    end
    if (early) fired <= True;
  endrule

  interface regs = r.regs;
  interface WdtPins pins;
    method Bit#(1) rst_out = (fired && r.ctrl_rstmode == 1) ? 1 : 0;
  endinterface
  method Bool irq = fired && r.ctrl_rstmode == 0;
endmodule

endpackage
