function out1 = F_uv_H2_coeffs(in1,in2,in3)
%F_UV_H2_COEFFS
%    OUT1 = F_UV_H2_COEFFS(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    03-Dec-2020 17:10:28

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
c22 = in3(22,:);
c23 = in3(23,:);
c24 = in3(24,:);
c25 = in3(25,:);
c26 = in3(26,:);
c27 = in3(27,:);
t2 = a11-a12;
t3 = a21-a22;
t4 = a31-a32;
t5 = t2.^2;
t6 = t3.^2;
t7 = t4.^2;
t8 = b2.^2;
t9 = b1.^2;
t10 = b3.^2;
t11 = a12-b1;
t12 = a22-b2;
t13 = a32-b3;
t14 = t11.^2;
t15 = t12.^2;
t16 = t13.^2;
out1 = [c22.*t5+c23.*t6+c24.*t7+b1.*c13.*t5.*3.0+b2.*c14.*t6.*3.0+b2.*c16.*t5+b1.*c18.*t6+b3.*c15.*t7.*3.0+b3.*c17.*t5+b1.*c20.*t7+b3.*c19.*t6+b2.*c21.*t7+c1.*t5.*t9.*6.0+c2.*t6.*t8.*6.0+c3.*t7.*t10.*6.0+c10.*t5.*t8+c10.*t6.*t9+c11.*t5.*t10+c11.*t7.*t9+c12.*t7.*t8+c12.*t6.*t10+c25.*t2.*t3+c26.*t2.*t4+c27.*t3.*t4+b1.*c16.*t2.*t3.*2.0+b1.*c17.*t2.*t4.*2.0+b2.*c18.*t2.*t3.*2.0+b2.*c19.*t3.*t4.*2.0+b3.*c20.*t2.*t4.*2.0+b3.*c21.*t3.*t4.*2.0+c4.*t2.*t3.*t9.*3.0+c6.*t2.*t3.*t8.*3.0+c5.*t2.*t4.*t9.*3.0+c7.*t3.*t4.*t8.*3.0+c8.*t2.*t4.*t10.*3.0+c9.*t3.*t4.*t10.*3.0+b1.*b2.*c4.*t5.*3.0+b1.*b3.*c5.*t5.*3.0+b1.*b2.*c6.*t6.*3.0+b2.*b3.*c7.*t6.*3.0+b1.*b3.*c8.*t7.*3.0+b2.*b3.*c9.*t7.*3.0+b1.*b2.*c10.*t2.*t3.*4.0+b1.*b3.*c11.*t2.*t4.*4.0+b2.*b3.*c12.*t3.*t4.*4.0,c22.*t2.*t11.*2.0+c23.*t3.*t12.*2.0+c25.*t2.*t12+c25.*t3.*t11+c24.*t4.*t13.*2.0+c26.*t2.*t13+c26.*t4.*t11+c27.*t3.*t13+c27.*t4.*t12+b1.*c13.*t2.*t11.*6.0+b1.*c16.*t2.*t12.*2.0+b1.*c16.*t3.*t11.*2.0+b2.*c14.*t3.*t12.*6.0+b2.*c16.*t2.*t11.*2.0+b1.*c17.*t2.*t13.*2.0+b1.*c17.*t4.*t11.*2.0+b3.*c17.*t2.*t11.*2.0+b1.*c18.*t3.*t12.*2.0+b2.*c18.*t2.*t12.*2.0+b2.*c18.*t3.*t11.*2.0+b3.*c15.*t4.*t13.*6.0+b2.*c19.*t3.*t13.*2.0+b2.*c19.*t4.*t12.*2.0+b3.*c19.*t3.*t12.*2.0+b1.*c20.*t4.*t13.*2.0+b3.*c20.*t2.*t13.*2.0+b3.*c20.*t4.*t11.*2.0+b2.*c21.*t4.*t13.*2.0+b3.*c21.*t3.*t13.*2.0+b3.*c21.*t4.*t12.*2.0+c1.*t2.*t9.*t11.*1.2e1+c2.*t3.*t8.*t12.*1.2e1+c4.*t2.*t9.*t12.*3.0+c4.*t3.*t9.*t11.*3.0+c6.*t2.*t8.*t12.*3.0+c6.*t3.*t8.*t11.*3.0+c5.*t2.*t9.*t13.*3.0+c5.*t4.*t9.*t11.*3.0+c3.*t4.*t10.*t13.*1.2e1+c7.*t3.*t8.*t13.*3.0+c7.*t4.*t8.*t12.*3.0+c10.*t2.*t8.*t11.*2.0+c8.*t2.*t10.*t13.*3.0+c8.*t4.*t10.*t11.*3.0+c10.*t3.*t9.*t12.*2.0+c11.*t2.*t10.*t11.*2.0+c9.*t3.*t10.*t13.*3.0+c9.*t4.*t10.*t12.*3.0+c11.*t4.*t9.*t13.*2.0+c12.*t3.*t10.*t12.*2.0+c12.*t4.*t8.*t13.*2.0+b1.*b2.*c4.*t2.*t11.*6.0+b1.*b3.*c5.*t2.*t11.*6.0+b1.*b2.*c6.*t3.*t12.*6.0+b1.*b2.*c10.*t2.*t12.*4.0+b1.*b2.*c10.*t3.*t11.*4.0+b2.*b3.*c7.*t3.*t12.*6.0+b1.*b3.*c8.*t4.*t13.*6.0+b1.*b3.*c11.*t2.*t13.*4.0+b1.*b3.*c11.*t4.*t11.*4.0+b2.*b3.*c9.*t4.*t13.*6.0+b2.*b3.*c12.*t3.*t13.*4.0+b2.*b3.*c12.*t4.*t12.*4.0,c22.*t14+c23.*t15+c24.*t16+b1.*c13.*t14.*3.0+b2.*c14.*t15.*3.0+b2.*c16.*t14+b1.*c18.*t15+b3.*c15.*t16.*3.0+b3.*c17.*t14+b1.*c20.*t16+b3.*c19.*t15+b2.*c21.*t16+c1.*t9.*t14.*6.0+c2.*t8.*t15.*6.0+c3.*t10.*t16.*6.0+c10.*t8.*t14+c10.*t9.*t15+c11.*t10.*t14+c11.*t9.*t16+c12.*t8.*t16+c12.*t10.*t15+c25.*t11.*t12+c26.*t11.*t13+c27.*t12.*t13+b1.*c16.*t11.*t12.*2.0+b1.*c17.*t11.*t13.*2.0+b2.*c18.*t11.*t12.*2.0+b2.*c19.*t12.*t13.*2.0+b3.*c20.*t11.*t13.*2.0+b3.*c21.*t12.*t13.*2.0+c4.*t9.*t11.*t12.*3.0+c6.*t8.*t11.*t12.*3.0+c5.*t9.*t11.*t13.*3.0+c7.*t8.*t12.*t13.*3.0+c8.*t10.*t11.*t13.*3.0+c9.*t10.*t12.*t13.*3.0+b1.*b2.*c4.*t14.*3.0+b1.*b3.*c5.*t14.*3.0+b1.*b2.*c6.*t15.*3.0+b2.*b3.*c7.*t15.*3.0+b1.*b3.*c8.*t16.*3.0+b2.*b3.*c9.*t16.*3.0+b1.*b2.*c10.*t11.*t12.*4.0+b1.*b3.*c11.*t11.*t13.*4.0+b2.*b3.*c12.*t12.*t13.*4.0];
