function [c,ceq]=sphereCollision(q, obstacles)
ceq=0;
%obstacles mx4 (x,y,z,r)
jointCoords=jointPoses(q); %%nx3,gives (x,y,z) coordinate of joint
A=zeros(size(jointCoords,1),size(obstacles,1));
jointCoords=[0,0,0;jointCoords];
for i=2:size(jointCoords,1)
    for j=1:size(obstacles,1)
        p=obstacles(j,1:3);
        v=jointCoords(i-1,:);
        w=jointCoords(i,:);
        d=norm(cross(w-v,p-v))/norm(w-v);
        if d<1.1*obstacles(j,4)
            t =dot(p - v, w - v) / norm(w-v);
            if t<norm(w-v)
                A(i,j)=1.1*obstacles(j,4)-d;
            else
                A(i,j)=1.1*obstacles(j,4)-sqrt((t-norm(w-v))^2+d^2);
            end
        else
            A(i,j)=1.1*obstacles(j,4)-d;
                
        end
    end
end
c=max((A).^3,[],2);