 %-------------------------------------------------------
% composes two transformations
%
% Author:  Jose Neira
% Version: 1.0, 7-Dic-2000
%-------------------------------------------------------
% History:
%-------------------------------------------------------
function tac=tcompm(tabm,tbc),

if size(tabm,1) ~= 3,
   error('TCOMP: tab is not a transformation!!!');
end;

if size(tbc,1) ~= 3,
   error('TCOMP: tbc is not a transformation!!!');
end;

result = AngleWrap(tabm(3,:)+tbc(3));


s = sin(tabm(3,:)');
c = cos(tabm(3,:)');

n = size(tabm,2);
Rm = reshape([c -s c s]',2,2*n)';
w = Rm*tbc(1:2);
tac = [tabm(1:2,:)+ reshape(w,2,n);
       result];