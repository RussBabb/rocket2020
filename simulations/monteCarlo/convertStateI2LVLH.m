function [x_LVLH, P_LVLH] = convertStateI2LVLH(x, P)
%convertStateI2LVLH Converts the position and velocity history of the
%satellite to LVLH
%   x - state vector time history
%   P - covariance time history
[m_x, n_x] = size(x);
[m_P, n_P, o_P] = size(P);
x_LVLH = zeros(6,n_x);
P_LVLH = zeros(6,6,o_P);

for i=1:n_x
    r = x(1:3,i);
    v = x(4:6,i);
    
    T_I2LVLH = calc_I2LVLH(r, v);
    r_LVLH = T_I2LVLH*r;
    v_LVLH = T_I2LVLH*v;
    
    x_LVLH(1:3,i) = r_LVLH;
    x_LVLH(1:3,i) = v_LVLH;
    
    T_I2LVLH6 = [T_I2LVLH, zeros(3,3); zeros(3,3), T_I2LVLH];
    P_LVLH(:,:,i) = T_I2LVLH6*P(1:6,1:6,i)*T_I2LVLH6';
end
    
end