function FK = FK_fn(in1)
%FK_FN
%    FK = FK_FN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    30-Jan-2017 16:09:24

rpy1_1 = in1(1,:);
rpy1_2 = in1(2,:);
rpy1_3 = in1(3,:);
rpy2_1 = in1(4,:);
rpy2_2 = in1(5,:);
rpy2_3 = in1(6,:);
rpy3_1 = in1(7,:);
rpy3_2 = in1(8,:);
rpy3_3 = in1(9,:);
t2 = rpy1_3.*(1.0./2.0);
t3 = rpy1_1.*(1.0./2.0);
t4 = rpy1_2.*(1.0./2.0);
t6 = cos(t3);
t7 = cos(t2);
t8 = sin(t4);
t9 = cos(t4);
t10 = sin(t3);
t11 = sin(t2);
t35 = t6.*t9.*t11;
t36 = t7.*t8.*t10;
t5 = t35+t36;
t37 = t6.*t7.*t8;
t38 = t9.*t10.*t11;
t12 = t37-t38;
t13 = rpy2_3.*(1.0./2.0);
t14 = rpy2_1.*(1.0./2.0);
t15 = rpy2_2.*(1.0./2.0);
t17 = cos(t14);
t18 = cos(t13);
t19 = sin(t15);
t20 = cos(t15);
t21 = sin(t14);
t22 = sin(t13);
t39 = t17.*t20.*t22;
t40 = t18.*t19.*t21;
t16 = t39+t40;
t41 = t17.*t18.*t19;
t42 = t20.*t21.*t22;
t23 = t41-t42;
t24 = rpy3_3.*(1.0./2.0);
t25 = rpy3_1.*(1.0./2.0);
t26 = rpy3_2.*(1.0./2.0);
t28 = cos(t25);
t29 = cos(t24);
t30 = sin(t26);
t31 = cos(t26);
t32 = sin(t25);
t33 = sin(t24);
t43 = t28.*t31.*t33;
t44 = t29.*t30.*t32;
t27 = t43+t44;
t45 = t28.*t29.*t30;
t46 = t31.*t32.*t33;
t34 = t45-t46;
t47 = t7.*t9.*t10;
t48 = t6.*t8.*t11;
t49 = t47+t48;
t50 = t6.*t7.*t9;
t51 = t18.*t20.*t21;
t52 = t17.*t19.*t22;
t53 = t51+t52;
t54 = t17.*t18.*t20;
t55 = t29.*t31.*t32;
t56 = t28.*t30.*t33;
t57 = t55+t56;
t58 = t28.*t29.*t31;
t61 = t19.*t21.*t22;
t59 = t54-t61;
t62 = t8.*t10.*t11;
t60 = t50-t62;
t75 = t30.*t32.*t33;
t63 = t58-t75;
t64 = t16.*t49;
t65 = t12.*t59;
t66 = t23.*t60;
t81 = t5.*t53;
t67 = t64+t65+t66-t81;
t68 = t5.*t16;
t69 = t49.*t53;
t70 = t12.*t23;
t80 = t59.*t60;
t71 = t68+t69+t70-t80;
t72 = t5.*t59;
t73 = t12.*t53;
t74 = t16.*t60;
t76 = t5.*t23;
t77 = t49.*t59;
t78 = t53.*t60;
t83 = t23.*t49;
t79 = t72+t73+t74-t83;
t82 = t76+t77+t78-t12.*t16;
FK = [t5.^2.*-6.0-t12.^2.*6.0-t16.^2.*4.0-t23.^2.*4.0-t27.^2.*2.0-t34.^2.*2.0+6.0;t12.*t49.*6.0-t5.*t60.*6.0-t16.*t59.*4.0+t23.*t53.*4.0-t27.*t63.*2.0+t34.*t57.*2.0;t5.*t49.*6.0+t16.*t53.*4.0+t12.*t60.*6.0+t23.*t59.*4.0+t27.*t57.*2.0+t34.*t63.*2.0;-t34.*t67-t27.*t79-t63.*t71-t57.*t82;-t27.*t67+t34.*t79-t57.*t71+t82.*(t58-t75);-t34.*t71+t27.*t82-t57.*t79+(t58-t75).*(t64+t65+t66-t81);-t27.*t71-t34.*t82+t57.*(t64+t65+t66-t81)+(t58-t75).*(t72+t73+t74-t83)];
