function cmd = unpack_openplc_commands(analogRegs,digitalCoils)
% convert hr101-hr106 and c51-c57 into engineering commands
validateattributes(analogRegs,{'numeric'},{'numel',6});
validateattributes(digitalCoils,{'numeric','logical'},{'numel',7});
cmd.P101A_speed_pct = analogRegs(1)/10;
cmd.P101B_speed_pct = analogRegs(2)/10;
cmd.P201_speed_pct = analogRegs(3)/10;
cmd.P301A_speed_pct = analogRegs(4)/10;
cmd.P301B_speed_pct = analogRegs(5)/10;
cmd.DP201_output_pct = analogRegs(6)/10;
cmd.P101A_start = logical(digitalCoils(1));
cmd.P101B_start = logical(digitalCoils(2));
cmd.P201_start = logical(digitalCoils(3));
cmd.P301A_start = logical(digitalCoils(4));
cmd.P301B_start = logical(digitalCoils(5));
cmd.M201_start = logical(digitalCoils(6));
cmd.XV201_open = logical(digitalCoils(7));
end
