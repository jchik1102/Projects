function regs = pack_openplc_plant_registers(LIT101_pct,LIT201_pct,LIT301_pct,FIT101_Lps,FIT201_Lps,FIT301_Lps,AIT201_mgL,PIT301_kPa,heartbeat)
%PACK_OPENPLC_PLANT_REGISTERS Create HR1-HR9 vector for Modbus Client Write.
regs = zeros(1,9);
regs(1) = min(max(round(LIT101_pct*10),0),1000);
regs(2) = min(max(round(LIT201_pct*10),0),1000);
regs(3) = min(max(round(LIT301_pct*10),0),1000);
regs(4) = min(max(round(FIT101_Lps*10),0),65535);
regs(5) = min(max(round(FIT201_Lps*10),0),65535);
regs(6) = min(max(round(FIT301_Lps*10),0),65535);
regs(7) = min(max(round(AIT201_mgL*100),0),65535);
regs(8) = min(max(round(PIT301_kPa*10),0),65535);
regs(9) = mod(round(heartbeat),65536);
end
