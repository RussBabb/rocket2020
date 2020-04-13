function C_D_comp = calcCDcomp(C_D_incomp, M)

if M < 0.8
    C_F_mkr = 1/sqrt(1 - M^2);
elseif M < 1
    C_F_mkr = -240.740740741*M^3 + 640.740740741*M^2 - 559.259259260*M + 162.259259259;
elseif M < 1.3
    C_F_mkr = 107.845968888*M^3 - 375.848824765*M^2 + 428.1597428660*M - 157.156886989;
else
    C_F_mkr = 1/sqrt(M^2 -1);
end

C_D_comp = C_F_mkr*C_D_incomp;
end