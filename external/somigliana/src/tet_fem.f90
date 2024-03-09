SUBROUTINE tet_linear(val, X, D, volume, lam, miu) 
IMPLICIT NONE 
REAL(KIND=8) val(1, 1) 
REAL(KIND=8) X(3, 4) 
REAL(KIND=8) D(3, 3) 
REAL(KIND=8) volume(1, 1) 
REAL(KIND=8) lam(1, 1) 
REAL(KIND=8) miu(1, 1) 
REAL(KIND=8)  tt1 
REAL(KIND=8)  tt2 
REAL(KIND=8)  tt3 
tt1 = -X(1,1)
tt2 = -X(2,1)
tt3 = -X(3,1)
val(1,1) = volume(1,1)*(5.0E-1*lam(1,1)*(5.0E-1*(2*D(3,3)*(X(3,4)&
&+tt3)+2*D(2,3)*(X(3,3)+tt3)+2*D(1,3)*(X(3,2)+tt3))+5.0E-1*(2*(X(2&
&,4)+tt2)*D(3,2)+2*D(2,2)*(X(2,3)+tt2)+2*D(1,2)*(X(2,2)+tt2))+5.0E&
&-1*(2*(X(1,4)+tt1)*D(3,1)+2*(X(1,3)+tt1)*D(2,1)+2*D(1,1)*(X(1,2)+&
&tt1))-3)**2+miu(1,1)*((1.0E+0*D(3,3)*X(3,4)+1.0E+0*D(2,3)*X(3,3)-&
&1.0E+0*X(3,1)*D(3,3)+1.0E+0*D(1,3)*X(3,2)-1.0E+0*D(2,3)*X(3,1)-1.&
&0E+0*D(1,3)*X(3,1)-1)**2+5.0E-1*(D(3,2)*X(3,4)+D(2,2)*X(3,3)+X(2,&
&4)*D(3,3)-X(2,1)*D(3,3)+D(1,2)*X(3,2)-X(3,1)*D(3,2)-D(2,2)*X(3,1)&
&-D(1,2)*X(3,1)+D(2,3)*X(2,3)-X(2,1)*D(2,3)+D(1,3)*X(2,2)-D(1,3)*X&
&(2,1))**2+5.0E-1*(D(3,1)*X(3,4)+D(2,1)*X(3,3)+X(1,4)*D(3,3)-X(1,1&
&)*D(3,3)+D(1,1)*X(3,2)-D(3,1)*X(3,1)-D(2,1)*X(3,1)-D(1,1)*X(3,1)+&
&X(1,3)*D(2,3)-X(1,1)*D(2,3)+X(1,2)*D(1,3)-X(1,1)*D(1,3))**2+(1.0E&
&+0*X(2,4)*D(3,2)-1.0E+0*X(2,1)*D(3,2)+1.0E+0*D(2,2)*X(2,3)+1.0E+0&
&*D(1,2)*X(2,2)-1.0E+0*X(2,1)*D(2,2)-1.0E+0*D(1,2)*X(2,1)-1)**2+5.&
&0E-1*(X(1,4)*D(3,2)-X(1,1)*D(3,2)+X(2,4)*D(3,1)-X(2,1)*D(3,1)+D(2&
&,1)*X(2,3)+D(1,1)*X(2,2)+X(1,3)*D(2,2)-X(1,1)*D(2,2)-D(2,1)*X(2,1&
&)-D(1,1)*X(2,1)+D(1,2)*X(1,2)-X(1,1)*D(1,2))**2+(1.0E+0*X(1,4)*D(&
&3,1)-1.0E+0*X(1,1)*D(3,1)+1.0E+0*X(1,3)*D(2,1)-1.0E+0*X(1,1)*D(2,&
&1)+1.0E+0*D(1,1)*X(1,2)-1.0E+0*D(1,1)*X(1,1)-1)**2))
END 
SUBROUTINE tet_linear_jac(jac, X, D, volume, lam, miu) 
IMPLICIT NONE 
REAL(KIND=8) jac(1, 12) 
REAL(KIND=8) X(3, 4) 
REAL(KIND=8) D(3, 3) 
REAL(KIND=8) volume(1, 1) 
REAL(KIND=8) lam(1, 1) 
REAL(KIND=8) miu(1, 1) 
REAL(KIND=8)  tt1 
REAL(KIND=8)  tt2 
REAL(KIND=8)  tt3 
REAL(KIND=8)  tt4 
REAL(KIND=8)  tt5 
REAL(KIND=8)  tt6 
REAL(KIND=8)  tt7 
REAL(KIND=8)  tt8 
REAL(KIND=8)  tt9 
REAL(KIND=8)  tt10 
REAL(KIND=8)  tt11 
REAL(KIND=8)  tt12 
REAL(KIND=8)  tt13 
tt1 = 1.0E+0*X(1,4)*D(3,1)-1.0E+0*X(1,1)*D(3,1)+1.0E+0*X(1,3)*D(2&
&,1)-1.0E+0*X(1,1)*D(2,1)+1.0E+0*D(1,1)*X(1,2)-1.0E+0*D(1,1)*X(1,1&
&)-1
tt2 = (-D(3,2))-D(2,2)-D(1,2)
tt3 = X(1,4)*D(3,2)-X(1,1)*D(3,2)+X(2,4)*D(3,1)-X(2,1)*D(3,1)+D(2&
&,1)*X(2,3)+D(1,1)*X(2,2)+X(1,3)*D(2,2)-X(1,1)*D(2,2)-D(2,1)*X(2,1&
&)-D(1,1)*X(2,1)+D(1,2)*X(1,2)-X(1,1)*D(1,2)
tt4 = (-D(3,3))-D(2,3)-D(1,3)
tt5 = D(3,1)*X(3,4)+D(2,1)*X(3,3)+X(1,4)*D(3,3)-X(1,1)*D(3,3)+D(1&
&,1)*X(3,2)-D(3,1)*X(3,1)-D(2,1)*X(3,1)-D(1,1)*X(3,1)+X(1,3)*D(2,3&
&)-X(1,1)*D(2,3)+X(1,2)*D(1,3)-X(1,1)*D(1,3)
tt6 = -X(1,1)
tt7 = -X(2,1)
tt8 = -X(3,1)
tt9 = 5.0E-1*(2*D(3,3)*(X(3,4)+tt8)+2*D(2,3)*(X(3,3)+tt8)+2*D(1,3&
&)*(X(3,2)+tt8))+5.0E-1*(2*(X(2,4)+tt7)*D(3,2)+2*D(2,2)*(X(2,3)+tt&
&7)+2*D(1,2)*(X(2,2)+tt7))+5.0E-1*(2*(X(1,4)+tt6)*D(3,1)+2*(X(1,3)&
&+tt6)*D(2,1)+2*D(1,1)*(X(1,2)+tt6))-3
tt10 = (-D(3,1))-D(2,1)-D(1,1)
tt11 = 1.0E+0*X(2,4)*D(3,2)-1.0E+0*X(2,1)*D(3,2)+1.0E+0*D(2,2)*X(&
&2,3)+1.0E+0*D(1,2)*X(2,2)-1.0E+0*X(2,1)*D(2,2)-1.0E+0*D(1,2)*X(2,&
&1)-1
tt12 = D(3,2)*X(3,4)+D(2,2)*X(3,3)+X(2,4)*D(3,3)-X(2,1)*D(3,3)+D(&
&1,2)*X(3,2)-X(3,1)*D(3,2)-D(2,2)*X(3,1)-D(1,2)*X(3,1)+D(2,3)*X(2,&
&3)-X(2,1)*D(2,3)+D(1,3)*X(2,2)-D(1,3)*X(2,1)
tt13 = 1.0E+0*D(3,3)*X(3,4)+1.0E+0*D(2,3)*X(3,3)-1.0E+0*X(3,1)*D(&
&3,3)+1.0E+0*D(1,3)*X(3,2)-1.0E+0*D(2,3)*X(3,1)-1.0E+0*D(1,3)*X(3,&
&1)-1
jac(1,1) = volume(1,1)*(5.0E-1*lam(1,1)*((-2*D(3,1))-2*D(2,1)-2*D&
&(1,1))*tt9+miu(1,1)*(1.0E+0*tt4*tt5+1.0E+0*tt2*tt3+2*((-1.0E+0*D(&
&3,1))-1.0E+0*D(2,1)-1.0E+0*D(1,1))*tt1))
jac(1,2) = volume(1,1)*(5.0E-1*lam(1,1)*((-2*D(3,2))-2*D(2,2)-2*D&
&(1,2))*tt9+miu(1,1)*(1.0E+0*tt4*tt12+2*((-1.0E+0*D(3,2))-1.0E+0*D&
&(2,2)-1.0E+0*D(1,2))*tt11+1.0E+0*tt10*tt3))
jac(1,3) = volume(1,1)*(5.0E-1*lam(1,1)*((-2*D(3,3))-2*D(2,3)-2*D&
&(1,3))*tt9+miu(1,1)*(2*((-1.0E+0*D(3,3))-1.0E+0*D(2,3)-1.0E+0*D(1&
&,3))*tt13+1.0E+0*tt2*tt12+1.0E+0*tt10*tt5))
jac(1,4) = volume(1,1)*(1.0E+0*D(1,1)*lam(1,1)*tt9+miu(1,1)*(1.0E&
&+0*D(1,3)*tt5+1.0E+0*D(1,2)*tt3+2.0E+0*D(1,1)*tt1))
jac(1,5) = volume(1,1)*(1.0E+0*lam(1,1)*D(1,2)*tt9+miu(1,1)*(1.0E&
&+0*D(1,3)*tt12+2.0E+0*D(1,2)*tt11+1.0E+0*D(1,1)*tt3))
jac(1,6) = volume(1,1)*(1.0E+0*lam(1,1)*D(1,3)*tt9+miu(1,1)*(2.0E&
&+0*D(1,3)*tt13+1.0E+0*D(1,2)*tt12+1.0E+0*D(1,1)*tt5))
jac(1,7) = volume(1,1)*(1.0E+0*lam(1,1)*D(2,1)*tt9+miu(1,1)*(1.0E&
&+0*D(2,3)*tt5+1.0E+0*D(2,2)*tt3+2.0E+0*D(2,1)*tt1))
jac(1,8) = volume(1,1)*(1.0E+0*lam(1,1)*D(2,2)*tt9+miu(1,1)*(1.0E&
&+0*D(2,3)*tt12+2.0E+0*D(2,2)*tt11+1.0E+0*D(2,1)*tt3))
jac(1,9) = volume(1,1)*(1.0E+0*lam(1,1)*D(2,3)*tt9+miu(1,1)*(2.0E&
&+0*D(2,3)*tt13+1.0E+0*D(2,2)*tt12+1.0E+0*D(2,1)*tt5))
jac(1,10) = volume(1,1)*(1.0E+0*lam(1,1)*D(3,1)*tt9+miu(1,1)*(1.0&
&E+0*D(3,3)*tt5+1.0E+0*D(3,2)*tt3+2.0E+0*D(3,1)*tt1))
jac(1,11) = volume(1,1)*(1.0E+0*lam(1,1)*D(3,2)*tt9+miu(1,1)*(1.0&
&E+0*D(3,3)*tt12+2.0E+0*D(3,2)*tt11+1.0E+0*D(3,1)*tt3))
jac(1,12) = volume(1,1)*(1.0E+0*lam(1,1)*D(3,3)*tt9+miu(1,1)*(2.0&
&E+0*D(3,3)*tt13+1.0E+0*D(3,2)*tt12+1.0E+0*D(3,1)*tt5))
END 
SUBROUTINE tet_linear_hes(hes, X, D, volume, lam, miu) 
IMPLICIT NONE 
REAL(KIND=8) hes(12, 12) 
REAL(KIND=8) X(3, 4) 
REAL(KIND=8) D(3, 3) 
REAL(KIND=8) volume(1, 1) 
REAL(KIND=8) lam(1, 1) 
REAL(KIND=8) miu(1, 1) 
REAL(KIND=8)  tt1 
REAL(KIND=8)  tt2 
REAL(KIND=8)  tt3 
REAL(KIND=8)  tt4 
REAL(KIND=8)  tt5 
REAL(KIND=8)  tt6 
REAL(KIND=8)  tt7 
REAL(KIND=8)  tt8 
REAL(KIND=8)  tt9 
REAL(KIND=8)  tt10 
REAL(KIND=8)  tt11 
REAL(KIND=8)  tt12 
REAL(KIND=8)  tt13 
REAL(KIND=8)  tt14 
REAL(KIND=8)  tt15 
REAL(KIND=8)  tt16 
REAL(KIND=8)  tt17 
REAL(KIND=8)  tt18 
REAL(KIND=8)  tt19 
REAL(KIND=8)  tt20 
REAL(KIND=8)  tt21 
REAL(KIND=8)  tt22 
REAL(KIND=8)  tt23 
REAL(KIND=8)  tt24 
REAL(KIND=8)  tt25 
REAL(KIND=8)  tt26 
REAL(KIND=8)  tt27 
REAL(KIND=8)  tt28 
REAL(KIND=8)  tt29 
REAL(KIND=8)  tt30 
REAL(KIND=8)  tt31 
REAL(KIND=8)  tt32 
REAL(KIND=8)  tt33 
REAL(KIND=8)  tt34 
REAL(KIND=8)  tt35 
REAL(KIND=8)  tt36 
REAL(KIND=8)  tt37 
REAL(KIND=8)  tt38 
REAL(KIND=8)  tt39 
REAL(KIND=8)  tt40 
REAL(KIND=8)  tt41 
REAL(KIND=8)  tt42 
REAL(KIND=8)  tt43 
REAL(KIND=8)  tt44 
REAL(KIND=8)  tt45 
REAL(KIND=8)  tt46 
REAL(KIND=8)  tt47 
REAL(KIND=8)  tt48 
REAL(KIND=8)  tt49 
REAL(KIND=8)  tt50 
REAL(KIND=8)  tt51 
REAL(KIND=8)  tt52 
REAL(KIND=8)  tt53 
REAL(KIND=8)  tt54 
REAL(KIND=8)  tt55 
REAL(KIND=8)  tt56 
REAL(KIND=8)  tt57 
REAL(KIND=8)  tt58 
REAL(KIND=8)  tt59 
REAL(KIND=8)  tt60 
REAL(KIND=8)  tt61 
REAL(KIND=8)  tt62 
REAL(KIND=8)  tt63 
REAL(KIND=8)  tt64 
REAL(KIND=8)  tt65 
REAL(KIND=8)  tt66 
REAL(KIND=8)  tt67 
REAL(KIND=8)  tt68 
REAL(KIND=8)  tt69 
REAL(KIND=8)  tt70 
REAL(KIND=8)  tt71 
REAL(KIND=8)  tt72 
REAL(KIND=8)  tt73 
REAL(KIND=8)  tt74 
REAL(KIND=8)  tt75 
REAL(KIND=8)  tt76 
REAL(KIND=8)  tt77 
REAL(KIND=8)  tt78 
REAL(KIND=8)  tt79 
REAL(KIND=8)  tt80 
REAL(KIND=8)  tt81 
REAL(KIND=8)  tt82 
REAL(KIND=8)  tt83 
REAL(KIND=8)  tt84 
REAL(KIND=8)  tt85 
REAL(KIND=8)  tt86 
REAL(KIND=8)  tt87 
REAL(KIND=8)  tt88 
REAL(KIND=8)  tt89 
REAL(KIND=8)  tt90 
REAL(KIND=8)  tt91 
REAL(KIND=8)  tt92 
REAL(KIND=8)  tt93 
REAL(KIND=8)  tt94 
REAL(KIND=8)  tt95 
REAL(KIND=8)  tt96 
REAL(KIND=8)  tt97 
REAL(KIND=8)  tt98 
REAL(KIND=8)  tt99 
REAL(KIND=8)  tt100 
REAL(KIND=8)  tt101 
REAL(KIND=8)  tt102 
REAL(KIND=8)  tt103 
REAL(KIND=8)  tt104 
REAL(KIND=8)  tt105 
REAL(KIND=8)  tt106 
REAL(KIND=8)  tt107 
REAL(KIND=8)  tt108 
REAL(KIND=8)  tt109 
REAL(KIND=8)  tt110 
REAL(KIND=8)  tt111 
REAL(KIND=8)  tt112 
REAL(KIND=8)  tt113 
REAL(KIND=8)  tt114 
tt1 = (-2*D(3,1))-2*D(2,1)-2*D(1,1)
tt2 = (-1.0E+0*D(3,1))-1.0E+0*D(2,1)-1.0E+0*D(1,1)
tt3 = (-D(3,2))-D(2,2)-D(1,2)
tt4 = 1.0E+0*tt3**2
tt5 = (-D(3,3))-D(2,3)-D(1,3)
tt6 = 1.0E+0*tt5**2
tt7 = (-2*D(3,2))-2*D(2,2)-2*D(1,2)
tt8 = (-D(3,1))-D(2,1)-D(1,1)
tt9 = volume(1,1)*(1.0E+0*miu(1,1)*tt8*tt3+2.5E-1*lam(1,1)*tt1*tt&
&7)
tt10 = (-2*D(3,3))-2*D(2,3)-2*D(1,3)
tt11 = volume(1,1)*(1.0E+0*miu(1,1)*tt8*tt5+2.5E-1*lam(1,1)*tt1*t&
&t10)
tt12 = 1.0E+0*D(1,2)*tt3
tt13 = 1.0E+0*D(1,3)*tt5
tt14 = volume(1,1)*(miu(1,1)*(tt13+tt12+2.0E+0*D(1,1)*tt2)+5.0E-1&
&*D(1,1)*lam(1,1)*tt1)
tt15 = volume(1,1)*(1.0E+0*D(1,1)*miu(1,1)*tt3+5.0E-1*lam(1,1)*D(&
&1,2)*tt1)
tt16 = volume(1,1)*(1.0E+0*D(1,1)*miu(1,1)*tt5+5.0E-1*lam(1,1)*D(&
&1,3)*tt1)
tt17 = 1.0E+0*D(2,2)*tt3
tt18 = 1.0E+0*D(2,3)*tt5
tt19 = volume(1,1)*(miu(1,1)*(tt18+tt17+2.0E+0*D(2,1)*tt2)+5.0E-1&
&*lam(1,1)*D(2,1)*tt1)
tt20 = volume(1,1)*(1.0E+0*miu(1,1)*D(2,1)*tt3+5.0E-1*lam(1,1)*D(&
&2,2)*tt1)
tt21 = volume(1,1)*(1.0E+0*miu(1,1)*D(2,1)*tt5+5.0E-1*lam(1,1)*D(&
&2,3)*tt1)
tt22 = 1.0E+0*tt3*D(3,2)
tt23 = 1.0E+0*tt5*D(3,3)
tt24 = volume(1,1)*(miu(1,1)*(tt23+tt22+2.0E+0*tt2*D(3,1))+5.0E-1&
&*lam(1,1)*tt1*D(3,1))
tt25 = volume(1,1)*(5.0E-1*lam(1,1)*tt1*D(3,2)+1.0E+0*miu(1,1)*D(&
&3,1)*tt3)
tt26 = volume(1,1)*(5.0E-1*lam(1,1)*tt1*D(3,3)+1.0E+0*miu(1,1)*D(&
&3,1)*tt5)
tt27 = 1.0E+0*tt8**2
tt28 = (-1.0E+0*D(3,2))-1.0E+0*D(2,2)-1.0E+0*D(1,2)
tt29 = volume(1,1)*(1.0E+0*miu(1,1)*tt3*tt5+2.5E-1*lam(1,1)*tt7*t&
&t10)
tt30 = volume(1,1)*(5.0E-1*D(1,1)*lam(1,1)*tt7+1.0E+0*miu(1,1)*D(&
&1,2)*tt8)
tt31 = 1.0E+0*D(1,1)*tt8
tt32 = volume(1,1)*(miu(1,1)*(tt13+2.0E+0*D(1,2)*tt28+tt31)+5.0E-&
&1*lam(1,1)*D(1,2)*tt7)
tt33 = volume(1,1)*(1.0E+0*miu(1,1)*D(1,2)*tt5+5.0E-1*lam(1,1)*D(&
&1,3)*tt7)
tt34 = volume(1,1)*(5.0E-1*lam(1,1)*D(2,1)*tt7+1.0E+0*miu(1,1)*D(&
&2,2)*tt8)
tt35 = 1.0E+0*D(2,1)*tt8
tt36 = volume(1,1)*(miu(1,1)*(tt18+2.0E+0*D(2,2)*tt28+tt35)+5.0E-&
&1*lam(1,1)*D(2,2)*tt7)
tt37 = volume(1,1)*(1.0E+0*miu(1,1)*D(2,2)*tt5+5.0E-1*lam(1,1)*D(&
&2,3)*tt7)
tt38 = volume(1,1)*(1.0E+0*miu(1,1)*tt8*D(3,2)+5.0E-1*lam(1,1)*D(&
&3,1)*tt7)
tt39 = 1.0E+0*tt8*D(3,1)
tt40 = volume(1,1)*(miu(1,1)*(tt23+2.0E+0*tt28*D(3,2)+tt39)+5.0E-&
&1*lam(1,1)*tt7*D(3,2))
tt41 = volume(1,1)*(5.0E-1*lam(1,1)*tt7*D(3,3)+1.0E+0*miu(1,1)*D(&
&3,2)*tt5)
tt42 = (-1.0E+0*D(3,3))-1.0E+0*D(2,3)-1.0E+0*D(1,3)
tt43 = volume(1,1)*(5.0E-1*D(1,1)*lam(1,1)*tt10+1.0E+0*miu(1,1)*D&
&(1,3)*tt8)
tt44 = volume(1,1)*(5.0E-1*lam(1,1)*D(1,2)*tt10+1.0E+0*miu(1,1)*D&
&(1,3)*tt3)
tt45 = volume(1,1)*(5.0E-1*lam(1,1)*D(1,3)*tt10+miu(1,1)*(2.0E+0*&
&D(1,3)*tt42+tt12+tt31))
tt46 = volume(1,1)*(5.0E-1*lam(1,1)*D(2,1)*tt10+1.0E+0*miu(1,1)*D&
&(2,3)*tt8)
tt47 = volume(1,1)*(5.0E-1*lam(1,1)*D(2,2)*tt10+1.0E+0*miu(1,1)*D&
&(2,3)*tt3)
tt48 = volume(1,1)*(5.0E-1*lam(1,1)*D(2,3)*tt10+miu(1,1)*(2.0E+0*&
&D(2,3)*tt42+tt17+tt35))
tt49 = volume(1,1)*(1.0E+0*miu(1,1)*tt8*D(3,3)+5.0E-1*lam(1,1)*D(&
&3,1)*tt10)
tt50 = volume(1,1)*(1.0E+0*miu(1,1)*tt3*D(3,3)+5.0E-1*lam(1,1)*D(&
&3,2)*tt10)
tt51 = volume(1,1)*(miu(1,1)*(2.0E+0*tt42*D(3,3)+tt22+tt39)+5.0E-&
&1*lam(1,1)*tt10*D(3,3))
tt52 = D(1,1)**2
tt53 = D(1,2)**2
tt54 = 1.0E+0*tt53
tt55 = D(1,3)**2
tt56 = 1.0E+0*tt55
tt57 = volume(1,1)*(1.0E+0*D(1,1)*miu(1,1)*D(1,2)+1.0E+0*D(1,1)*l&
&am(1,1)*D(1,2))
tt58 = volume(1,1)*(1.0E+0*D(1,1)*miu(1,1)*D(1,3)+1.0E+0*D(1,1)*l&
&am(1,1)*D(1,3))
tt59 = 1.0E+0*D(1,2)*D(2,2)
tt60 = 1.0E+0*D(1,3)*D(2,3)
tt61 = volume(1,1)*(miu(1,1)*(tt60+tt59+2.0E+0*D(1,1)*D(2,1))+1.0&
&E+0*D(1,1)*lam(1,1)*D(2,1))
tt62 = volume(1,1)*(1.0E+0*D(1,1)*lam(1,1)*D(2,2)+1.0E+0*miu(1,1)&
&*D(1,2)*D(2,1))
tt63 = volume(1,1)*(1.0E+0*D(1,1)*lam(1,1)*D(2,3)+1.0E+0*miu(1,1)&
&*D(1,3)*D(2,1))
tt64 = 1.0E+0*D(1,2)*D(3,2)
tt65 = 1.0E+0*D(1,3)*D(3,3)
tt66 = volume(1,1)*(miu(1,1)*(tt65+tt64+2.0E+0*D(1,1)*D(3,1))+1.0&
&E+0*D(1,1)*lam(1,1)*D(3,1))
tt67 = volume(1,1)*(1.0E+0*D(1,1)*lam(1,1)*D(3,2)+1.0E+0*miu(1,1)&
&*D(1,2)*D(3,1))
tt68 = volume(1,1)*(1.0E+0*D(1,1)*lam(1,1)*D(3,3)+1.0E+0*miu(1,1)&
&*D(1,3)*D(3,1))
tt69 = 1.0E+0*tt52
tt70 = volume(1,1)*(1.0E+0*miu(1,1)*D(1,2)*D(1,3)+1.0E+0*lam(1,1)&
&*D(1,2)*D(1,3))
tt71 = volume(1,1)*(1.0E+0*D(1,1)*miu(1,1)*D(2,2)+1.0E+0*lam(1,1)&
&*D(1,2)*D(2,1))
tt72 = 1.0E+0*D(1,1)*D(2,1)
tt73 = volume(1,1)*(miu(1,1)*(tt60+2.0E+0*D(1,2)*D(2,2)+tt72)+1.0&
&E+0*lam(1,1)*D(1,2)*D(2,2))
tt74 = volume(1,1)*(1.0E+0*lam(1,1)*D(1,2)*D(2,3)+1.0E+0*miu(1,1)&
&*D(1,3)*D(2,2))
tt75 = volume(1,1)*(1.0E+0*D(1,1)*miu(1,1)*D(3,2)+1.0E+0*lam(1,1)&
&*D(1,2)*D(3,1))
tt76 = 1.0E+0*D(1,1)*D(3,1)
tt77 = volume(1,1)*(miu(1,1)*(tt65+2.0E+0*D(1,2)*D(3,2)+tt76)+1.0&
&E+0*lam(1,1)*D(1,2)*D(3,2))
tt78 = volume(1,1)*(1.0E+0*lam(1,1)*D(1,2)*D(3,3)+1.0E+0*miu(1,1)&
&*D(1,3)*D(3,2))
tt79 = volume(1,1)*(1.0E+0*D(1,1)*miu(1,1)*D(2,3)+1.0E+0*lam(1,1)&
&*D(1,3)*D(2,1))
tt80 = volume(1,1)*(1.0E+0*miu(1,1)*D(1,2)*D(2,3)+1.0E+0*lam(1,1)&
&*D(1,3)*D(2,2))
tt81 = volume(1,1)*(miu(1,1)*(2.0E+0*D(1,3)*D(2,3)+tt59+tt72)+1.0&
&E+0*lam(1,1)*D(1,3)*D(2,3))
tt82 = volume(1,1)*(1.0E+0*D(1,1)*miu(1,1)*D(3,3)+1.0E+0*lam(1,1)&
&*D(1,3)*D(3,1))
tt83 = volume(1,1)*(1.0E+0*miu(1,1)*D(1,2)*D(3,3)+1.0E+0*lam(1,1)&
&*D(1,3)*D(3,2))
tt84 = volume(1,1)*(miu(1,1)*(2.0E+0*D(1,3)*D(3,3)+tt64+tt76)+1.0&
&E+0*lam(1,1)*D(1,3)*D(3,3))
tt85 = D(2,1)**2
tt86 = D(2,2)**2
tt87 = 1.0E+0*tt86
tt88 = D(2,3)**2
tt89 = 1.0E+0*tt88
tt90 = volume(1,1)*(1.0E+0*miu(1,1)*D(2,1)*D(2,2)+1.0E+0*lam(1,1)&
&*D(2,1)*D(2,2))
tt91 = volume(1,1)*(1.0E+0*miu(1,1)*D(2,1)*D(2,3)+1.0E+0*lam(1,1)&
&*D(2,1)*D(2,3))
tt92 = 1.0E+0*D(2,2)*D(3,2)
tt93 = 1.0E+0*D(2,3)*D(3,3)
tt94 = volume(1,1)*(miu(1,1)*(tt93+tt92+2.0E+0*D(2,1)*D(3,1))+1.0&
&E+0*lam(1,1)*D(2,1)*D(3,1))
tt95 = volume(1,1)*(1.0E+0*lam(1,1)*D(2,1)*D(3,2)+1.0E+0*miu(1,1)&
&*D(2,2)*D(3,1))
tt96 = volume(1,1)*(1.0E+0*lam(1,1)*D(2,1)*D(3,3)+1.0E+0*miu(1,1)&
&*D(2,3)*D(3,1))
tt97 = 1.0E+0*tt85
tt98 = volume(1,1)*(1.0E+0*miu(1,1)*D(2,2)*D(2,3)+1.0E+0*lam(1,1)&
&*D(2,2)*D(2,3))
tt99 = volume(1,1)*(1.0E+0*miu(1,1)*D(2,1)*D(3,2)+1.0E+0*lam(1,1)&
&*D(2,2)*D(3,1))
tt100 = 1.0E+0*D(2,1)*D(3,1)
tt101 = volume(1,1)*(miu(1,1)*(tt93+2.0E+0*D(2,2)*D(3,2)+tt100)+1&
&.0E+0*lam(1,1)*D(2,2)*D(3,2))
tt102 = volume(1,1)*(1.0E+0*lam(1,1)*D(2,2)*D(3,3)+1.0E+0*miu(1,1&
&)*D(2,3)*D(3,2))
tt103 = volume(1,1)*(1.0E+0*miu(1,1)*D(2,1)*D(3,3)+1.0E+0*lam(1,1&
&)*D(2,3)*D(3,1))
tt104 = volume(1,1)*(1.0E+0*miu(1,1)*D(2,2)*D(3,3)+1.0E+0*lam(1,1&
&)*D(2,3)*D(3,2))
tt105 = volume(1,1)*(miu(1,1)*(2.0E+0*D(2,3)*D(3,3)+tt92+tt100)+1&
&.0E+0*lam(1,1)*D(2,3)*D(3,3))
tt106 = D(3,1)**2
tt107 = D(3,2)**2
tt108 = 1.0E+0*tt107
tt109 = D(3,3)**2
tt110 = 1.0E+0*tt109
tt111 = volume(1,1)*(1.0E+0*miu(1,1)*D(3,1)*D(3,2)+1.0E+0*lam(1,1&
&)*D(3,1)*D(3,2))
tt112 = volume(1,1)*(1.0E+0*miu(1,1)*D(3,1)*D(3,3)+1.0E+0*lam(1,1&
&)*D(3,1)*D(3,3))
tt113 = 1.0E+0*tt106
tt114 = volume(1,1)*(1.0E+0*miu(1,1)*D(3,2)*D(3,3)+1.0E+0*lam(1,1&
&)*D(3,2)*D(3,3))
hes(1,1) = volume(1,1)*(miu(1,1)*(tt6+tt4+2*tt2**2)+2.5E-1*lam(1,&
&1)*tt1**2)
hes(1,2) = tt9
hes(1,3) = tt11
hes(1,4) = tt14
hes(1,5) = tt15
hes(1,6) = tt16
hes(1,7) = tt19
hes(1,8) = tt20
hes(1,9) = tt21
hes(1,10) = tt24
hes(1,11) = tt25
hes(1,12) = tt26
hes(2,1) = tt9
hes(2,2) = volume(1,1)*(miu(1,1)*(tt6+2*tt28**2+tt27)+2.5E-1*lam(&
&1,1)*tt7**2)
hes(2,3) = tt29
hes(2,4) = tt30
hes(2,5) = tt32
hes(2,6) = tt33
hes(2,7) = tt34
hes(2,8) = tt36
hes(2,9) = tt37
hes(2,10) = tt38
hes(2,11) = tt40
hes(2,12) = tt41
hes(3,1) = tt11
hes(3,2) = tt29
hes(3,3) = volume(1,1)*(miu(1,1)*(2*tt42**2+tt4+tt27)+2.5E-1*lam(&
&1,1)*tt10**2)
hes(3,4) = tt43
hes(3,5) = tt44
hes(3,6) = tt45
hes(3,7) = tt46
hes(3,8) = tt47
hes(3,9) = tt48
hes(3,10) = tt49
hes(3,11) = tt50
hes(3,12) = tt51
hes(4,1) = tt14
hes(4,2) = tt30
hes(4,3) = tt43
hes(4,4) = volume(1,1)*(miu(1,1)*(tt56+tt54+2.0E+0*tt52)+1.0E+0*t&
&t52*lam(1,1))
hes(4,5) = tt57
hes(4,6) = tt58
hes(4,7) = tt61
hes(4,8) = tt62
hes(4,9) = tt63
hes(4,10) = tt66
hes(4,11) = tt67
hes(4,12) = tt68
hes(5,1) = tt15
hes(5,2) = tt32
hes(5,3) = tt44
hes(5,4) = tt57
hes(5,5) = volume(1,1)*(miu(1,1)*(tt56+2.0E+0*tt53+tt69)+1.0E+0*l&
&am(1,1)*tt53)
hes(5,6) = tt70
hes(5,7) = tt71
hes(5,8) = tt73
hes(5,9) = tt74
hes(5,10) = tt75
hes(5,11) = tt77
hes(5,12) = tt78
hes(6,1) = tt16
hes(6,2) = tt33
hes(6,3) = tt45
hes(6,4) = tt58
hes(6,5) = tt70
hes(6,6) = volume(1,1)*(miu(1,1)*(2.0E+0*tt55+tt54+tt69)+1.0E+0*l&
&am(1,1)*tt55)
hes(6,7) = tt79
hes(6,8) = tt80
hes(6,9) = tt81
hes(6,10) = tt82
hes(6,11) = tt83
hes(6,12) = tt84
hes(7,1) = tt19
hes(7,2) = tt34
hes(7,3) = tt46
hes(7,4) = tt61
hes(7,5) = tt71
hes(7,6) = tt79
hes(7,7) = volume(1,1)*(miu(1,1)*(tt89+tt87+2.0E+0*tt85)+1.0E+0*l&
&am(1,1)*tt85)
hes(7,8) = tt90
hes(7,9) = tt91
hes(7,10) = tt94
hes(7,11) = tt95
hes(7,12) = tt96
hes(8,1) = tt20
hes(8,2) = tt36
hes(8,3) = tt47
hes(8,4) = tt62
hes(8,5) = tt73
hes(8,6) = tt80
hes(8,7) = tt90
hes(8,8) = volume(1,1)*(miu(1,1)*(tt89+2.0E+0*tt86+tt97)+1.0E+0*l&
&am(1,1)*tt86)
hes(8,9) = tt98
hes(8,10) = tt99
hes(8,11) = tt101
hes(8,12) = tt102
hes(9,1) = tt21
hes(9,2) = tt37
hes(9,3) = tt48
hes(9,4) = tt63
hes(9,5) = tt74
hes(9,6) = tt81
hes(9,7) = tt91
hes(9,8) = tt98
hes(9,9) = volume(1,1)*(miu(1,1)*(2.0E+0*tt88+tt87+tt97)+1.0E+0*l&
&am(1,1)*tt88)
hes(9,10) = tt103
hes(9,11) = tt104
hes(9,12) = tt105
hes(10,1) = tt24
hes(10,2) = tt38
hes(10,3) = tt49
hes(10,4) = tt66
hes(10,5) = tt75
hes(10,6) = tt82
hes(10,7) = tt94
hes(10,8) = tt99
hes(10,9) = tt103
hes(10,10) = volume(1,1)*(miu(1,1)*(tt110+tt108+2.0E+0*tt106)+1.0&
&E+0*lam(1,1)*tt106)
hes(10,11) = tt111
hes(10,12) = tt112
hes(11,1) = tt25
hes(11,2) = tt40
hes(11,3) = tt50
hes(11,4) = tt67
hes(11,5) = tt77
hes(11,6) = tt83
hes(11,7) = tt95
hes(11,8) = tt101
hes(11,9) = tt104
hes(11,10) = tt111
hes(11,11) = volume(1,1)*(miu(1,1)*(tt110+2.0E+0*tt107+tt113)+1.0&
&E+0*lam(1,1)*tt107)
hes(11,12) = tt114
hes(12,1) = tt26
hes(12,2) = tt41
hes(12,3) = tt51
hes(12,4) = tt68
hes(12,5) = tt78
hes(12,6) = tt84
hes(12,7) = tt96
hes(12,8) = tt102
hes(12,9) = tt105
hes(12,10) = tt112
hes(12,11) = tt114
hes(12,12) = volume(1,1)*(miu(1,1)*(2.0E+0*tt109+tt108+tt113)+1.0&
&E+0*lam(1,1)*tt109)
END 
