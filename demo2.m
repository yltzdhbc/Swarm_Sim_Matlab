close all;
fol_num=4;        
N=5;             % 4follower and 1 leader
countmax=2000;
dt=0.1;
gama=0.65;%机器人之间的影响因子，过大容易造成过冲而抖动
beta=13;%障碍物影响因子
K0=1;
KN=0.2;
goal=[25 25];
m_count = 0;
is_arrive = 0;
% x最高速度m/s],y最高速度[m/s],x最高加速度[m/ss],y最高加速度[m/ss]]
Kinematic=[0.7;0.7;0.4;0.4];
error_distance = [0;0;0;0];
color='ybgcrkr'; %%%定义颜色标记
type=[2,1,0.5,0.5,2,2];%%%定义线的类型
start_time = clock;
%% 1-4行为follower 最后一行为leader
% A=[0 1 1 1 1;     % a(ij)
%    0 0 0 0 1;
%    0 0 0 1 1;
%    0 0 1 0 1;
%    0 0 0 0 0];
A=[0 0 0 0 1;     % a(ij)%%只考虑前面机器人的影响
   1 0 0 0 1;
   0 0 0 0 1;
   0 0 1 0 1;
   0 0 0 0 0];
 %% 初始化 位置pose、速度V、加速度控制量control
%         init_f=[-4.5 -1.5 0;%%%[x y th]
%                 -6 -1.5 pi/4; 
%                 -4.5 -4.5 -pi/4;
%                 -6 -4.5 pi/2;
%                 -3 -3 0];   
        init_f=[0 0.5 pi/4;%%%[x y th] %%队形切换 启动
                0 1 pi/4; 
                0 -0.5 pi/4;
                0 -1 pi/4;
                0 0 pi/4];  
    pose_x=init_f(:,1);
    pose_y=init_f(:,2);
    pose_th=init_f(:,3);
%     ob_temp=[-10 1.2;
%              -10 2;
%              -10 12];
    %%障碍物坐标[x y]
    ob_temp=[5 4; 5 8;8 5;];
%     ob_temp=ob_temp';
    %% follower相对leader的位置
%     delta_x=[-1.5 -3 -1.5 -3 0];   % 相对间隔误差   
%     delta_y=[1.5 1.5 -1.5 -1.5 0];  %领航者与自己无误差
    delta_x=[-1.5 -3 0 0 0];   % 相对间隔误差   
    delta_y=[0 0 -1.5 -3 0];  %领航者与自己无误差
    V_x(:,1)=[0;0;0;0;0];
    V_y(:,1)=[0;0;0;0;0]; %%%leader在y方向的初始速度为1m/s
    k=0;
    d_max=2;
    detect_R=1;
    ideal_posex=init_f(:,1);
    ideal_posey=init_f(:,2);
    %% 开始循环 走顺时针圆周
    for count=1:countmax
        if count == 415  %队形切换
           delta_x=[-1 -3 -2 -4 0];   % 相对间隔误差   
           delta_y=[-1 -3 -2 -4 0];  %领航者与自己无误差
        end
        if count == 620  %队形切换
           delta_x=[-1.5 -3 0 0 0];   % 相对间隔误差   
           delta_y=[0 0 -1.5 -3 0];  %领航者与自己无误差
        end
        k=k+1;
%         %%%做直线
%         V_x(N,k+1)=V_x(N,k);
%         V_y(N,k+1)=V_y(N,k);
%         %%%做圆周
%         V_x(N,k+1)=cos(k*dt);
%         V_y(N,k+1)=sin(k*dt);
        %%%朝目标点运动
        distance=sqrt((goal(1)-pose_x(N,k))^2+(goal(2)-pose_y(N,k))^2);%领导者距离目标点的距离
        th=atan2(goal(2)-pose_y(N,k),goal(1)-pose_x(N,k));%领导者与目标点之间的角度差
        if distance>2   %将最大距离设置为2
            distance=2;
        end
%         if distance<0.3   %将最大距离设置为2
%             distance=0.3;
%         end
        V_x(N,k+1)=KN*distance*cos(th); %%设置x,y方向的速度
        V_y(N,k+1)=KN*distance*sin(th);
        mse_leader=0;
        if(rem(k,5)==1&&k>1)    %%暂时不知道rem是什么，没用
            ideal_posex(N,(k-1)/5+1)=V_x(N,k+1)*dt*5+pose_x(N,k);
            ideal_posey(N,(k-1)/5+1)=V_y(N,k+1)*dt*5+pose_y(N,k);
        end
        %% 领航者避障
        %%%考虑冲突避免加上斥力
%         kk=0;
%         for j=1:N-1
%             kk=kk+1;
%             obs_pose(kk,1)=pose_x(j,k);
%             obs_pose(kk,2)=pose_y(j,k);
%         end
%         ob_pose=[obs_pose;ob_temp];
        ob_pose=ob_temp;
        repulsion=compute_repulsion([pose_x(N,k),pose_y(N,k)],ob_pose,detect_R);        
        %%%%%
        V_x(N,k+1)=V_x(N,k+1)+beta*repulsion(1);
        V_y(N,k+1)=V_y(N,k+1)+beta*repulsion(2);
        
         %%%出现局部极小的情况施加随机扰动 
        if(distance>1&&abs(V_x(N,k+1))<=0.1&&abs(V_y(N,k+1))<=0.1)
%             V_x(N,k+1)=beta*(1+rand(1))*repulsion(1);
%             V_y(N,k+1)=beta*(1+rand(1))*repulsion(2);
            V_x(N,k+1)=-1+2*rand(1);
            V_y(N,k+1)=-1+2*rand(1);
        end
        %%跟随者运动
        for i=1:fol_num  %fol_num=4      
            sum_delta_x=0;
            sum_delta_y=0;
            for j=1:N %%考虑邻居对它的影响
                sum_delta_x=sum_delta_x+A(i,j)*((pose_x(j,k)-pose_x(i,k))-(delta_x(j)-delta_x(i)));
                sum_delta_y=sum_delta_y+A(i,j)*((pose_y(j,k)-pose_y(i,k))-(delta_y(j)-delta_y(i)));   
            end
%             distance=[];
            error_distance(i,k+1)=sqrt(sum_delta_x^2+ sum_delta_y^2);
            th=atan2(sum_delta_y, sum_delta_x);
%             if error_distance(i,k+1)>d_max
%                 error_distance(i,k+1)=d_max;
%             end
            V_x(i,k+1)=gama*error_distance(i,k+1)*cos(th);
            V_y(i,k+1)=gama*error_distance(i,k+1)*sin(th);
%             disp(['i is',num2str(i)]);%打印distance
%             disp(['distance is',num2str(distance(i,k+1))]);%打印distance
%             disp(['V_x1 is',num2str(V_x(1,k+1))]);
%             disp(['V_y1 is',num2str(V_y(1,k+1))]);
            if(rem(k,5)==1&&k>1)
                ideal_posex(i,(k-1)/5+1)=V_x(i,k+1)*dt*5+pose_x(i,k);
                ideal_posey(i,(k-1)/5+1)=V_y(i,k+1)*dt*5+pose_y(i,k);
            end
           %%%考虑冲突避免加上斥力
            kk=0;
            for j=1:N
                if j~=i
                    kk=kk+1;
                    obs_pose(kk,1)=pose_x(j,k);
                    obs_pose(kk,2)=pose_y(j,k);
                end
            end
            ob_pose=[obs_pose;ob_temp];
            repulsion=compute_repulsion([pose_x(i,k),pose_y(i,k)],ob_pose,detect_R);        
            %%%%%
            V_x(i,k+1)=K0*V_x(N,k)+V_x(i,k+1)+beta*repulsion(1);
            V_y(i,k+1)=K0*V_y(N,k)+V_y(i,k+1)+beta*repulsion(2);
            %%%跟随着出现局部极小的情况施加随机扰动 
            if(error_distance(i,k+1)>0.5&&abs(V_x(i,k+1))<=0.1&&abs(V_y(i,k+1))<=0.1&&distance>1)
                V_x(i,k+1)=-1+2*rand(1);
                V_y(i,k+1)=-1+2*rand(1);
                disp(['distance is',num2str(error_distance(i,k+1))]);%打印distance
                disp(['rand V_x is',num2str(V_x(i,k+1))]);
                disp(['rand V_y is',num2str(V_y(i,k+1))]);
            end
% %             out=confine([V_x(i,k) V_y(i,k)],[V_x(i,k+1) V_y(i,k+1)],Kinematic);
% %             V_x(i,k+1)=out(1);
% %             V_y(i,k+1)=out(2);
        end
        %%
        for i=1:N
            out=confine([V_x(i,k) V_y(i,k)],[V_x(i,k+1) V_y(i,k+1)],Kinematic,0.1);
%             out=[V_x(i,k+1) V_y(i,k+1)];
            V_x(i,k+1)=out(1);
            V_y(i,k+1)=out(2);
            pose_x(i,k+1)=pose_x(i,k)+dt*V_x(i,k+1);
            pose_y(i,k+1)=pose_y(i,k)+dt*V_y(i,k+1);
            pose_th(i,k+1)=atan2(V_y(i,k+1),V_x(i,k+1));
        end
        tt_x(1:4,k)=pose_x(5,k);
        error_x(:,k)=tt_x(1:4,k)-pose_x(1:4,k)+(delta_x(1:4))';
        tt_y(1:4,k)=pose_y(5,k);
        error_y(:,k)=tt_y(1:4,k)-pose_y(1:4,k)+(delta_y(1:4))';
        %% ====Animation====
        area = compute_area(pose_x(N,k+1),pose_y(N,k+1),10);
        hold off;
        ArrowLength=0.7;% 箭头长度
        for j=1:N
            quiver(pose_x(j,k+1),pose_y(j,k+1),ArrowLength*cos(pose_th(j,k+1)),ArrowLength*sin(pose_th(j,k+1)),'.','color',color(1,j),'LineWidth',1.3);hold on;
            draw_circle(pose_x(j,k+1),pose_y(j,k+1),0.1,j);hold on;
        end
        obn = size(ob_temp);
        for i =1:obn
            draw_square(ob_temp(i,1),ob_temp(i,2),0.2);hold on;
        end
        xlabel('x Position(m)');
        ylabel('y Position(m)');
%         plot(ob_temp(:,1),ob_temp(:,2),'^k','LineWidth',2);hold on;
        x1 = [3,7,6,2];
        y1 = [5,9,10,6];
        x2 = [5,9,10,6];
        y2 = [3,7,6,2];
        x1=x1+8;y1=y1+8;x2=x2+8;y2=y2+8;
        fill(x1,y1,'k')             % 画填充图，填充区域为绿色
        fill(x2,y2,'k')             % 画填充图，填充区域为绿色
%         area=[-10 10 -10 10];
        axis(area);
        grid on;
        drawnow;    
        %% 判断终止条件
        now=[pose_x(N,k+1),pose_y(N,k+1)];
        if norm(now-goal)<0.2
            is_arrive = 1;
            m_count = m_count + 1;
            %队形切换 驻停
            delta_x=[-1 -2 1 2 0];   % 相对间隔误差   
            delta_y=[0 0 0 0 0];  %领航者与自己无误差
        end
        if m_count > 100
           end_time = clock;
           disp('Arrive Goal!!');break;
        end
    end
    %% 画图
    figure                               
    for i=1:N  %%路径
        plot(pose_x(i,:),pose_y(i,:),color(1,i),'LineWidth',1.5);
        hold on
    end
    for i=1:N  %%机器人起点和路径中机器人位置
        plot(pose_x(i,1),pose_y(i,1),'p','color',color(1,i),'LineWidth',2);
        hold on
        draw_circle(pose_x(i,300),pose_y(i,300),0.2,i);hold on;
        draw_circle(pose_x(i,570),pose_y(i,570),0.2,i);hold on;
        draw_circle(pose_x(i,760),pose_y(i,760),0.2,i);hold on;
    end
%     plot(pose_x(N,1),pose_y(N,1),'p','color',color(1,N),'LineWidth',2);
%     hold on
    for i=1:N  %%机器人终点
        plot(pose_x(i,k),pose_y(i,k),'h','color',color(1,i),'LineWidth',2);
        hold on
    end
%     plot(pose_x(N,k),pose_y(N,k),'h','color',color(1,N),'LineWidth',2);
%     hold on
    for i =1:obn
        draw_square(ob_temp(i,1),ob_temp(i,2),0.2);hold on;
    end
    %plot(ob_temp(:,1),ob_temp(:,2),'Xk','LineWidth',2);hold on;
    grid on;
    fill(x1,y1,'k')             % 画填充图，填充区域为绿色
    fill(x2,y2,'k')             % 画填充图，填充区域为绿色
    xlabel('x');
    ylabel('y');
    legend('follower1','follower2','follower3','follower4','leader','Location','NorthWest');
    xlabel('x Position(m)');
    ylabel('y Position(m)');
    title('基于拓扑图与跟随领导者法的五机器人编队控制算法');
    %% 画误差图
    cost_time = 3600*(end_time(4)-start_time(4)) + 60 * (end_time(5)-start_time(5)) + (end_time(6) - start_time(6));
    kx=cost_time/k;
    cx=0:kx:cost_time;
    figure                                %   生成三维平面图  连续
    error=sqrt(error_x.^2+error_y.^2);
    for i=1:4
        plot(cx(1:k-1),error(i,1:k-1),color(1,i),'LineWidth',1.5);
        hold on;
    end
    legend('follower1','follower2','follower3','follower4');
    xlabel('时间(s)');
    ylabel('位置误差(m)');
    title('五机器人编队控制各机器人仿真误差曲线');
    
function [ next] = confine(current,next,Kinematic,dt)
%%%current=[v_x v_y];
%%%%Kinematic=[ x最高速度m/s],y最高速度[m/s],x最高加速度[m/ss],y最高加速度[m/ss]]
%%%Kinematic=[1;1;0.5;0.5];
%% 速度x上的限制
delta_x=next(1)-current(1);
if delta_x>=0
    next(1)=min(current(1)+delta_x,current(1)+Kinematic(3)*dt);
else
    next(1)=max(current(1)+delta_x,current(1)-Kinematic(3)*dt);
end
if next(1)>=0
    next(1)=min(next(1),Kinematic(1));
else
    next(1)=max(next(1),-Kinematic(1));
end
%% 速度y上的限制
delta_y=next(2)-current(2);
if delta_y>=0
    next(2)=min(current(2)+delta_y,current(2)+Kinematic(4)*dt);
else
    next(2)=max(current(2)+delta_y,current(2)-Kinematic(4)*dt);
end
if next(2)>=0
    next(2)=min(next(2),Kinematic(2));
else
    next(2)=max(next(2),-Kinematic(2));
end
end

