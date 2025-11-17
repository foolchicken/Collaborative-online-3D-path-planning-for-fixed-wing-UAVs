function angle = VectorAngle(A,B)
% 计算向量A逆时针转到向量B的夹角，angle范围为0到±pi
if numel(A) == 2
    dir = A(1)*B(2) - A(2)*B(1);
else
    dir = sum(cross(A,B));
end
temp1 = norm(A)*norm(B);
if temp1<1e-10,temp1 = 1e-10;end
temp2 = sum(A.*B)/temp1;
if temp2>1,temp2 = 1;end
if temp2<-1,temp2 = -1;end
angle = acos( temp2 );
if dir<0, angle = -angle;end
% if angle<0
%     angle = 2*pi+angle;
% end