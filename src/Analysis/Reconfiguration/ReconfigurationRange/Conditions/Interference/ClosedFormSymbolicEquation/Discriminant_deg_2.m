function u_c = Discriminant_deg_2(in1,in2,u0)
%DISCRIMINANT_DEG_2
%    U_C = DISCRIMINANT_DEG_2(IN1,IN2,U0)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    03-Dec-2020 17:11:18

u11 = in2(:,1);
u12 = in2(:,2);
u21 = in1(:,1);
u22 = in1(:,2);
u23 = in1(:,3);
u_c = [u0.*u21.*-4.0+u11.^2,u0.*u22.*-4.0+u11.*u12.*2.0,u0.*u23.*-4.0+u12.^2];