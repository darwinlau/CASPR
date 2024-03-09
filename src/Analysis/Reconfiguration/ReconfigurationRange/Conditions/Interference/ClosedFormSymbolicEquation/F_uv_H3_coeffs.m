function out1 = F_uv_H3_coeffs(in1,in2,in3)
%F_UV_H3_COEFFS
%    OUT1 = F_UV_H3_COEFFS(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    03-Dec-2020 17:10:27

a11 = in2(1);
a12 = in2(4);
a21 = in2(2);
a22 = in2(5);
a31 = in2(3);
a32 = in2(6);
b1 = in1(1,:);
b2 = in1(2,:);
b3 = in1(3,:);
c1 = in3(1,:);
c2 = in3(2,:);
c3 = in3(3,:);
c4 = in3(4,:);
c5 = in3(5,:);
c6 = in3(6,:);
c7 = in3(7,:);
c8 = in3(8,:);
c9 = in3(9,:);
c10 = in3(10,:);
c11 = in3(11,:);
c12 = in3(12,:);
c13 = in3(13,:);
c14 = in3(14,:);
c15 = in3(15,:);
c16 = in3(16,:);
c17 = in3(17,:);
c18 = in3(18,:);
c19 = in3(19,:);
c20 = in3(20,:);
c21 = in3(21,:);
t2 = a11-a12;
t3 = t2.^2;
t4 = a21-a22;
t5 = t4.^2;
t6 = a31-a32;
t7 = t6.^2;
t8 = a12-b1;
t9 = a22-b2;
t10 = a32-b3;
t11 = t8.^2;
t12 = t9.^2;
t13 = t10.^2;
out1 = [c13.*t2.*t3+c14.*t4.*t5+c16.*t3.*t4+c18.*t2.*t5+c17.*t3.*t6+c15.*t6.*t7+c20.*t2.*t7+c19.*t5.*t6+c21.*t4.*t7+b1.*c1.*t2.*t3.*4.0+b2.*c4.*t2.*t3+b1.*c4.*t3.*t4.*3.0+b2.*c2.*t4.*t5.*4.0+b3.*c5.*t2.*t3+b1.*c5.*t3.*t6.*3.0+b2.*c6.*t2.*t5.*3.0+b1.*c6.*t4.*t5+b1.*c10.*t2.*t5.*2.0+b2.*c10.*t3.*t4.*2.0+b3.*c3.*t6.*t7.*4.0+b3.*c7.*t4.*t5+b2.*c7.*t5.*t6.*3.0+b3.*c8.*t2.*t7.*3.0+b1.*c11.*t2.*t7.*2.0+b1.*c8.*t6.*t7+b3.*c9.*t4.*t7.*3.0+b3.*c11.*t3.*t6.*2.0+b2.*c9.*t6.*t7+b2.*c12.*t4.*t7.*2.0+b3.*c12.*t5.*t6.*2.0,c13.*t3.*t8.*3.0+c14.*t5.*t9.*3.0+c16.*t3.*t9+c17.*t3.*t10+c18.*t5.*t8+c15.*t7.*t10.*3.0+c19.*t5.*t10+c20.*t7.*t8+c21.*t7.*t9+b1.*c1.*t3.*t8.*1.2e1+b1.*c4.*t3.*t9.*3.0+b2.*c4.*t3.*t8.*3.0+b2.*c2.*t5.*t9.*1.2e1+b1.*c5.*t3.*t10.*3.0+b3.*c5.*t3.*t8.*3.0+b1.*c6.*t5.*t9.*3.0+b2.*c6.*t5.*t8.*3.0+b3.*c3.*t7.*t10.*1.2e1+b1.*c10.*t5.*t8.*2.0+b2.*c7.*t5.*t10.*3.0+b2.*c10.*t3.*t9.*2.0+b3.*c7.*t5.*t9.*3.0+b1.*c8.*t7.*t10.*3.0+b3.*c8.*t7.*t8.*3.0+b1.*c11.*t7.*t8.*2.0+b3.*c11.*t3.*t10.*2.0+b2.*c9.*t7.*t10.*3.0+b3.*c9.*t7.*t9.*3.0+b2.*c12.*t7.*t9.*2.0+b3.*c12.*t5.*t10.*2.0+c16.*t2.*t4.*t8.*2.0+c17.*t2.*t6.*t8.*2.0+c18.*t2.*t4.*t9.*2.0+c19.*t4.*t6.*t9.*2.0+c20.*t2.*t6.*t10.*2.0+c21.*t4.*t6.*t10.*2.0+b1.*c4.*t2.*t4.*t8.*6.0+b1.*c5.*t2.*t6.*t8.*6.0+b2.*c6.*t2.*t4.*t9.*6.0+b1.*c10.*t2.*t4.*t9.*4.0+b2.*c10.*t2.*t4.*t8.*4.0+b2.*c7.*t4.*t6.*t9.*6.0+b3.*c8.*t2.*t6.*t10.*6.0+b1.*c11.*t2.*t6.*t10.*4.0+b3.*c11.*t2.*t6.*t8.*4.0+b3.*c9.*t4.*t6.*t10.*6.0+b2.*c12.*t4.*t6.*t10.*4.0+b3.*c12.*t4.*t6.*t9.*4.0,c13.*t2.*t11.*3.0+c14.*t4.*t12.*3.0+c16.*t4.*t11+c18.*t2.*t12+c15.*t6.*t13.*3.0+c17.*t6.*t11+c20.*t2.*t13+c19.*t6.*t12+c21.*t4.*t13+b1.*c1.*t2.*t11.*1.2e1+b2.*c4.*t2.*t11.*3.0+b1.*c4.*t4.*t11.*3.0+b2.*c2.*t4.*t12.*1.2e1+b3.*c5.*t2.*t11.*3.0+b2.*c6.*t2.*t12.*3.0+b1.*c5.*t6.*t11.*3.0+b1.*c6.*t4.*t12.*3.0+b1.*c10.*t2.*t12.*2.0+b3.*c3.*t6.*t13.*1.2e1+b3.*c7.*t4.*t12.*3.0+b3.*c8.*t2.*t13.*3.0+b1.*c11.*t2.*t13.*2.0+b2.*c7.*t6.*t12.*3.0+b2.*c10.*t4.*t11.*2.0+b1.*c8.*t6.*t13.*3.0+b3.*c9.*t4.*t13.*3.0+b2.*c9.*t6.*t13.*3.0+b2.*c12.*t4.*t13.*2.0+b3.*c11.*t6.*t11.*2.0+b3.*c12.*t6.*t12.*2.0+c16.*t2.*t8.*t9.*2.0+c17.*t2.*t8.*t10.*2.0+c18.*t4.*t8.*t9.*2.0+c19.*t4.*t9.*t10.*2.0+c20.*t6.*t8.*t10.*2.0+c21.*t6.*t9.*t10.*2.0+b1.*c4.*t2.*t8.*t9.*6.0+b1.*c5.*t2.*t8.*t10.*6.0+b2.*c6.*t4.*t8.*t9.*6.0+b2.*c10.*t2.*t8.*t9.*4.0+b1.*c10.*t4.*t8.*t9.*4.0+b2.*c7.*t4.*t9.*t10.*6.0+b3.*c11.*t2.*t8.*t10.*4.0+b3.*c8.*t6.*t8.*t10.*6.0+b1.*c11.*t6.*t8.*t10.*4.0+b3.*c9.*t6.*t9.*t10.*6.0+b3.*c12.*t4.*t9.*t10.*4.0+b2.*c12.*t6.*t9.*t10.*4.0,c13.*t8.*t11+c14.*t9.*t12+c16.*t9.*t11+c15.*t10.*t13+c17.*t10.*t11+c18.*t8.*t12+c19.*t10.*t12+c20.*t8.*t13+c21.*t9.*t13+b1.*c1.*t8.*t11.*4.0+b1.*c4.*t9.*t11.*3.0+b2.*c2.*t9.*t12.*4.0+b2.*c4.*t8.*t11+b1.*c5.*t10.*t11.*3.0+b3.*c5.*t8.*t11+b1.*c6.*t9.*t12+b2.*c6.*t8.*t12.*3.0+b3.*c3.*t10.*t13.*4.0+b1.*c10.*t8.*t12.*2.0+b2.*c7.*t10.*t12.*3.0+b3.*c7.*t9.*t12+b1.*c8.*t10.*t13+b2.*c10.*t9.*t11.*2.0+b3.*c8.*t8.*t13.*3.0+b1.*c11.*t8.*t13.*2.0+b2.*c9.*t10.*t13+b3.*c9.*t9.*t13.*3.0+b3.*c11.*t10.*t11.*2.0+b2.*c12.*t9.*t13.*2.0+b3.*c12.*t10.*t12.*2.0];