function [ output_args ] = defend( input_args )
%%%验证一阶一致性算法 完整型约束 有机器人相互避障及物体避障，并通过机器学习实现被攻击机器人的检测
%%% 不考虑直接的速度约束 
%%% 2019-2-27
close all;
clear;clc;
fol_num=4;        
N=5;             % 4follower and 1 leader
countmax=800;
dt=0.1;
gama=0.5;
beta=10;
K0=1;
KN=0.2;
goal=[20 20];
sum_pose_x=0;
sum_pose_y=0;
label=[0;0;0;0;0];%用于存放受攻击的标签
flag_att=0;%检测是否受到攻击
q=1;%q为attacker重点攻击的对象
% x最高速度m/s],y最高旋转速度[rad/s],x最高加速度[m/ss],y最高加速度[rad/ss]]
Kinematic=[0.4;0.4;0.4;0.4];
%% 1-4行为follower 最后一行为leader
A=[0 1 1 1 1;     % a(ij)
   0 0 0 0 1;
   0 0 0 1 1;
   0 0 1 0 1;
   0 0 0 0 0];
 %% 初始化 位置pose、速度V、加速度控制量control
%     init_f=[-3 -3 0; %%%[x y th]
%                 -4 2 0;
%                 2 4 pi/4; 
%                 8 -3 -pi/4;
%                 -1 -1 pi/2];           
    init_f=[-4.5 -1.5 0;%%%[x y th]
                -6 -1 pi/4; 
                -2 -3.5 -pi/4;
                -6 -4 pi/2;
                -2.5 -3 0];      
    pose_x=init_f(:,1);
    pose_y=init_f(:,2);
    pose_th=init_f(:,3);
    ideal_posex=init_f(:,1);
    ideal_posey=init_f(:,2);
    init_att=[6 3 0];
    pose_x_att=init_att(:,1);
    pose_y_att=init_att(:,2);
    pose_th_att=init_att(:,3);
    pose_y_attpurpose=[];
    pose_x_attpurpose=[];
    ob_temp=[3 2;
             5 2;
             4 10];
%     ob_temp=[-10 1.2;
%              -10 2;
%              -10 12];
%     ob_temp=ob_temp';
    %% follower相对leader的位置
    delta_x=[-1.5 -3 -1.5 -3 0];   % 相对间隔误差   
    delta_y=[1.5 1.5 -1.5 -1.5 0];  %领航者与自己无误差
    V_x(:,1)=[0;0;0;0;0];
    V_y(:,1)=[0;0;0;0;0]; %%%leader在y方向的初始速度为1m/s
    Vx_att(:,1)=[0];
    Vy_att(:,1)=[0];
    k=0;
    d_max=2;
    detect_R=1;
    att_detect_R=0.6;
    attmse(:,1) = [0;0;0;0;0;0];
    %% 开始循环 走顺时针圆周
    for count=1:countmax
        k=k+1;
        if(k>100&&rem(k,20)==1)
            if abs(attmse(6,(k-6)/5)-attmse(6,(k-36)/5))<0.03
                q=rem(q+1,5)+1
            end
        end
%         %%%做直线
%         V_x(N,k+1)=V_x(N,k);
%         V_y(N,k+1)=V_y(N,k);
%         %%%做圆周
%         V_x(N,k+1)=cos(k*dt);
%         V_y(N,k+1)=sin(k*dt);
        %%%朝目标点运动
        distance=sqrt((goal(1)-pose_x(N,k))^2+(goal(2)-pose_y(N,k))^2);
        th=atan2(goal(2)-pose_y(N,k),goal(1)-pose_x(N,k));
        if distance>2
            distance=2;
        end
        V_x(N,k+1)=KN*distance*cos(th);
        V_y(N,k+1)=KN*distance*sin(th);
        %计算leader的mse
        mse_leader=0;
%         pose_x(N,k)
        if(rem(k,5)==1&&k>1)
            ideal_posex(N,(k-1)/5+1)=V_x(N,k+1)*dt*5+pose_x(N,k);
            ideal_posey(N,(k-1)/5+1)=V_y(N,k+1)*dt*5+pose_y(N,k);
        end
%         mse_leader = cal_mse([pose_x(N,k),pose_y(N,k)],[ideal_posex(N,k),ideal_posey(N,k)])
%         attmse(N,k)=mse_leader;
        %% 领航者避障
        %%%考虑冲突避免加上斥力
        ob_pose=[[pose_x_att(k),pose_y_att(k)];ob_temp];
        repulsion=compute_repulsion([pose_x(N,k),pose_y(N,k)],ob_pose,detect_R);        
        %%%%%
        V_x(N,k+1)=V_x(N,k+1)+beta*repulsion(1);
        V_y(N,k+1)=V_y(N,k+1)+beta*repulsion(2);
         %%%出现局部极小的情况施加扰动
        if(distance>0.1&&abs(V_x(N,k+1))<=0.1&&abs(V_y(N,k+1))<=0.1)
%             V_x(N,k+1)=beta*(1+rand(1))*repulsion(1);
%             V_y(N,k+1)=beta*(1+rand(1))*repulsion(2);
            V_x(N,k+1)=-1+2*rand(1);
            V_y(N,k+1)=-1+2*rand(1);
        end
        
%攻击者的控制算法----------------------------------
        sum_pose_x=0;%计算所有观测节点的坐标之和
        sum_pose_y=0;
        att_mse=[];
        for j=1:5
            sum_pose_x=pose_x(j,k)+sum_pose_x;
            sum_pose_y=pose_y(j,k)+sum_pose_y;
        end
        if(rem(k,5)==1&&k>1)
            attmse(6,(k-1)/5)=0;
            for j=1:fol_num
                if flag_att==1
                    switch j
                        case 1
                            att_mse(5) = cal_mse([pose_x(5,k),pose_y(5,k)],[ideal_posex(5,(k-1)/5),ideal_posey(5,(k-1)/5)]);
                            attmse(5,(k-1)/5) = abs(att_mse(5)-0.2);
                            attmse(6,(k-1)/5) = attmse(6,(k-1)/5) + abs(att_mse(5)-0.2);
                        otherwise
                            att_mse(j) = cal_mse([pose_x(j,k),pose_y(j,k)],[ideal_posex(j,(k-1)/5),ideal_posey(j,(k-1)/5)]);
                            attmse(j,(k-1)/5) = abs(att_mse(j)-0.2);
                            attmse(6,(k-1)/5) = attmse(6,(k-1)/5) + abs(att_mse(j)-0.2);
                    end
                else
                    att_mse(j) = cal_mse([pose_x(j,k),pose_y(j,k)],[ideal_posex(j,(k-1)/5),ideal_posey(j,(k-1)/5)]);
                    attmse(j,(k-1)/5) = abs(att_mse(j)-0.2);
                    attmse(6,(k-1)/5) = attmse(6,(k-1)/5) + abs(att_mse(j)-0.2);
                end
            end
            if(attmse(6, (k-1)/5)> 0.3)
                pose_x_attpurpose(k+1)=sum_pose_x/N;
                pose_y_attpurpose(k+1)=sum_pose_y/N;
            else
%                 pose_x_attpurpose(k+1)=(ideal_posex(1,(k-1)/5)+ideal_posex(2,(k-1)/5))/2+1;
%                 pose_y_attpurpose(k+1)=(ideal_posey(1,(k-1)/5)+ideal_posey(2,(k-1)/5))/2+1;
                pose_x_attpurpose(k+1)=ideal_posex(5,(k-1)/5)+1;
                pose_y_attpurpose(k+1)=ideal_posey(5,(k-1)/5)+1;
            end
        elseif(k==1)
            pose_x_attpurpose(k+1)=sum_pose_x/N;
            pose_y_attpurpose(k+1)=sum_pose_y/N;
        else
            pose_x_attpurpose(k+1)=pose_x_attpurpose(k);
            pose_y_attpurpose(k+1)=pose_y_attpurpose(k);
        end
         %攻击者下一时刻的目标位置及目标速度
%         pose_x_attpurpose(k+1)=sum_pose_x/N;
%         pose_y_attpurpose(k+1)=sum_pose_y/N;
        
        delta_att_x=pose_x_attpurpose(k)-pose_x_att(k);
        delta_att_y=pose_y_attpurpose(k)-pose_y_att(k);
        att_th=atan2(delta_att_y, delta_att_x);%计算攻击者与目标点位的角度Th
        dist_att=sqrt(delta_att_x^2 + delta_att_y^2);
        if dist_att>d_max
            dist_att=d_max;
        end
        Vx_att(k+1)=dist_att*gama*cos(att_th)*1.2;
        Vy_att(k+1)=dist_att*gama*sin(att_th)*1.2;
        %为攻击者加入避障算法
        avo_k=0;
        att_obs_pose=[];
        for i=1:5
            avo_k=avo_k+1;
            att_obs_pose(avo_k,1)=pose_x(i,k);
            att_obs_pose(avo_k,2)=pose_y(i,k);
        end
        att_ob_pose=[att_obs_pose;ob_temp];
%         att_ob_pose=ob_temp;
        att_repulsion=compute_repulsion([pose_x_att(k),pose_y_att(k)],att_ob_pose,att_detect_R);        
        %%%%加入避障后更改攻击者的速度
        Vx_att(k+1)=Vx_att(k+1)+beta*att_repulsion(1)*0.8;
        Vy_att(k+1)=Vy_att(k+1)+beta*att_repulsion(2)*0.8;
%---------------------------------------------------

%followers的控制算法----------------------------------
    if flag_att==1
        for i=2:5 
            sum_delta_x=0;
            sum_delta_y=0;
            for j=1:5 %%考虑邻居对它的影响
                sum_delta_x=sum_delta_x+A(i,j)*((pose_x(j,k)-pose_x(i,k))-(delta_x(j)-delta_x(i)));
                sum_delta_y=sum_delta_y+A(i,j)*((pose_y(j,k)-pose_y(i,k))-(delta_y(j)-delta_y(i)));  
            end
            
            distance=sqrt(sum_delta_x^2+ sum_delta_y^2);
            th=atan2(sum_delta_y, sum_delta_x);
            if distance>d_max
                distance=d_max;
            end
            V_x(i,k+1)=gama*distance*cos(th);
            V_y(i,k+1)=gama*distance*sin(th);
            if(rem(k,5)==1&&k>1)
                ideal_posex(i,(k-1)/5+1)=V_x(i,k+1)*dt*5+pose_x(i,k);
                ideal_posey(i,(k-1)/5+1)=V_y(i,k+1)*dt*5+pose_y(i,k);
            end
           %%%考虑冲突避免加上斥力
            kk=0;
            for j=1:5
                if j~=i
                    kk=kk+1;
                    obs_pose(kk,1)=pose_x(j,k);
                    obs_pose(kk,2)=pose_y(j,k);
                end
            end
            %加上攻击者的影响
            obs_pose(kk+1,1)=pose_x_att(k);
            obs_pose(kk+1,2)=pose_y_att(k);
            ob_pose=[obs_pose;ob_temp];
            repulsion=compute_repulsion([pose_x(i,k),pose_y(i,k)],ob_pose,detect_R);        
            %%%%%
            V_x(i,k+1)=K0*V_x(N,k)+V_x(i,k+1)+beta*repulsion(1);
            V_y(i,k+1)=K0*V_y(N,k)+V_y(i,k+1)+beta*repulsion(2);
% %             out=confine([V_x(i,k) V_y(i,k)],[V_x(i,k+1) V_y(i,k+1)],Kinematic);
% %             V_x(i,k+1)=out(1);
% %             V_y(i,k+1)=out(2);
        end
    else
        for i=1:fol_num  
            sum_delta_x=0;
            sum_delta_y=0;
            for j=1:5 %%考虑邻居对它的影响
                sum_delta_x=sum_delta_x+A(i,j)*((pose_x(j,k)-pose_x(i,k))-(delta_x(j)-delta_x(i)));
                sum_delta_y=sum_delta_y+A(i,j)*((pose_y(j,k)-pose_y(i,k))-(delta_y(j)-delta_y(i)));  
            end
            
            distance=sqrt(sum_delta_x^2+ sum_delta_y^2);
            th=atan2(sum_delta_y, sum_delta_x);
            if distance>d_max
                distance=d_max;
            end
            V_x(i,k+1)=gama*distance*cos(th);
            V_y(i,k+1)=gama*distance*sin(th);
            if(rem(k,5)==1&&k>1)
                ideal_posex(i,(k-1)/5+1)=V_x(i,k+1)*dt*5+pose_x(i,k);
                ideal_posey(i,(k-1)/5+1)=V_y(i,k+1)*dt*5+pose_y(i,k);
            end
           %%%考虑冲突避免加上斥力
            kk=0;
            for j=1:5
                if j~=i
                    kk=kk+1;
                    obs_pose(kk,1)=pose_x(j,k);
                    obs_pose(kk,2)=pose_y(j,k);
                end
            end
            %加上攻击者的影响
            obs_pose(kk+1,1)=pose_x_att(k);
            obs_pose(kk+1,2)=pose_y_att(k);
            ob_pose=[obs_pose;ob_temp];
            repulsion=compute_repulsion([pose_x(i,k),pose_y(i,k)],ob_pose,detect_R);        
            %%%%%
            V_x(i,k+1)=K0*V_x(N,k)+V_x(i,k+1)+beta*repulsion(1);
            V_y(i,k+1)=K0*V_y(N,k)+V_y(i,k+1)+beta*repulsion(2);
% %             out=confine([V_x(i,k) V_y(i,k)],[V_x(i,k+1) V_y(i,k+1)],Kinematic);
% %             V_x(i,k+1)=out(1);
% %             V_y(i,k+1)=out(2);
        end
    end
        for i=1:5
            out=confine([V_x(i,k) V_y(i,k)],[V_x(i,k+1) V_y(i,k+1)],Kinematic,0.1);
%             out=[V_x(i,k+1) V_y(i,k+1)];
            V_x(i,k+1)=out(1);
            V_y(i,k+1)=out(2);
            pose_x(i,k+1)=pose_x(i,k)+dt*V_x(i,k+1);
            pose_y(i,k+1)=pose_y(i,k)+dt*V_y(i,k+1);
            pose_th(i,k+1)=atan2(V_y(i,k+1),V_x(i,k+1));
        end
        if(rem(k,5)==1&&k>1)
            mse_leader = cal_mse([pose_x(N,k),pose_y(N,k)],[ideal_posex(N,(k-1)/5),ideal_posey(N,(k-1)/5)]);
            attmse(N,(k-1)/5)=mse_leader;
        end
        %计算攻击者下一时刻的位置
        pose_x_att(k+1)=pose_x_att(k)+dt*Vx_att(k+1);
        pose_y_att(k+1)=pose_y_att(k)+dt*Vy_att(k+1);
        pose_th_att(k+1)=atan2(Vy_att(k+1),Vx_att(k+1));
        %------------------------------
        if flag_att==0
            tt_x(1:4,k)=pose_x(5,k);
            error_x(:,k)=tt_x(1:4,k)-pose_x(1:4,k)+(delta_x(1:4))';
            tt_y(1:4,k)=pose_y(5,k);
            error_y(:,k)=tt_y(1:4,k)-pose_y(1:4,k)+(delta_y(1:4))';
        else
            tt_x(2:5,k)=pose_x(1,k);
            error_x(:,k)=tt_x(2:5,k)-pose_x(2:5,k)+(delta_x(2:5))';
            tt_y(2:5,k)=pose_y(1,k);
            error_y(:,k)=tt_y(2:5,k)-pose_y(2:5,k)+(delta_y(2:5))';
        end
        if(k==100)
            bbb=1;
        end
        %% ====Animation====
        area = compute_area(pose_x(N,k+1),pose_y(N,k+1),6);
        hold off;
        ArrowLength=0.5;% 
        for j=1:5
            if flag_att==1 && label(j)==1
                quiver(pose_x(j,k+1),pose_y(j,k+1),ArrowLength*cos(pose_th(j,k+1)),ArrowLength*sin(pose_th(j,k+1)),'*k');hold on;
                draw_circle (pose_x(j,k+1),pose_y(j,k+1),0.25,'-c');hold on;
            else
                quiver(pose_x(j,k+1),pose_y(j,k+1),ArrowLength*cos(pose_th(j,k+1)),ArrowLength*sin(pose_th(j,k+1)),'*k');hold on;
                draw_circle (pose_x(j,k+1),pose_y(j,k+1),0.25);hold on;
            end
        end
        %画出攻击者的变化
        quiver(pose_x_att(k+1),pose_y_att(k+1),ArrowLength*cos(pose_th_att(k+1)),ArrowLength*sin(pose_th_att(k+1)),'*k');hold on;
        draw_circle2 (pose_x_att(k+1),pose_y_att(k+1),0.25,'-r');hold on;
        
        plot(ob_temp(:,1),ob_temp(:,2),'Xk','LineWidth',2);hold on;
%         area=[-10 10 -10 10];
        axis(area);
        grid on;
        drawnow;    
        %% 判断终止条件
        now=[pose_x(N,k+1),pose_y(N,k+1)];
        if norm(now-goal)<0.5
            disp('Arrive Goal!!');break;
        end
        if k==500
            A=[0 0 0 0 0;
                1 0 0 0 0;
                1 1 0 0 0;
                1 0 1 0 0;
                0 1 1 1 1;];
            N=1;
            for i=1:5
                dmax(i)=max(attmse(i,1:99));
            end
            attmse(:,100)=[0;0;0;0;1;0];
            for i=1:5
                if(dmax(i)>0.1)%排除一些噪声进行逐行归一化
                    attmse(i,1:99)=normalization(attmse(i,1:99),0,dmax(i),0,1);
                else
                    attmse(i,1:99)=normalization(attmse(i,1:99),0,max(dmax(:)),0,1);
                end
            end
            save('attmse.mat','attmse');
%             label=svm(1);   %用学习出来的model检测哪个机器人受到了攻击
            flag_att=1;
        end
    end
    N=5;
    
    %% 画图
    %数据归一化处理
%     for i=1:5
%         dmax(i)=max(attmse(i,1:99));
%     end
%     for i=1:5
%         if(dmax(i)>0.1)%排除一些噪声进行逐行归一化
%             attmse(i,1:99)=normalization(attmse(i,1:99),0,dmax(i),0,1);
%         else
%             attmse(i,1:99)=normalization(attmse(i,1:99),0,max(dmax(:)),0,1);
%         end
%     end
%     save('attmse.mat','attmse');
%     label=svm(1)   %用学习出来的model检测哪个机器人受到了攻击
%     load('data.mat')
%     b=attmse(1:5,:);
%     a=[a;b];
%     save('data.mat','a');
%     xlswrite('attmse.xlsx',attmse);%写入xlsx文件进行外部归一化
    color='mgbkrc'; %%%定义颜色标记
    type=[2,1,0.5,0.5,2,2];%%%定义线的类型
    figure                               
    for i=1:5
        plot(pose_x(i,:),pose_y(i,:),color(1,i),'LineWidth',2);
        hold on
    end
    plot(pose_x_att(:),pose_y_att(:),color(1,N+1),'LineWidth',2);
    hold on
    for i=1:4
        plot(pose_x(i,1),pose_y(i,1),'bp','color',color(1,i),'LineWidth',1);
        hold on
    end
    plot(pose_x(N,1),pose_y(N,1),'*','color',color(1,N),'LineWidth',1);
    hold on
    plot(pose_x_att(1),pose_y_att(1),'*','color',color(1,6),'LineWidth',1);
    hold on
    for i=1:4
        plot(pose_x(i,k),pose_y(i,k),'m^','color',color(1,i),'LineWidth',2);
        hold on
    end
    plot(pose_x(N,k),pose_y(N,k),'o','color',color(1,N),'LineWidth',2);
    hold on
    plot(pose_x_att(k),pose_y_att(k),'o','color',color(1,6),'LineWidth',2);
    hold on
    plot(ob_temp(:,1),ob_temp(:,2),'Xk','LineWidth',2);hold on;
    grid on;
    xlabel('x');
    ylabel('y');
    legend('跟随者1','跟随者2','跟随者3','跟随者4','领航者','攻击者');
    xlabel('x(m)');
    ylabel('y(m)');
    title('对基于一致性的一阶编队算法的攻击');
    %% 画误差图
    cx=0:0.1:k/10;
    figure                                %   生成三维平面图  连续
    error=sqrt(error_x.^2+error_y.^2);
    for i=1:4
        plot(cx(1:k-1),error(i,1:k-1),color(1,i));
        hold on;
    end
    legend('跟随者1','跟随者2','跟随者3','跟随者4');
    xlabel('时间(s)');
    ylabel('位置误差');
    title('有攻击情况下各跟随者与领航者的误差曲线');
%     attmse(N+1,:)
    cx=0:0.5:k/10;
    figure                                %   生成三维平面图  连续
    for i=1:5
        plot(cx(1:k/5-1),attmse(i,1:(length(attmse))),color(1,i),'LineWidth',type(i));
        hold on;
    end
    legend('跟随者1','跟随者2','跟随者3','跟随者4','领航者');
    xlabel('时间(s)');
    ylabel('AO');
    title('有攻击情况下各机器人的AO曲线');
    cx=0:0.5:k/10;
    figure                                %   生成三维平面图  连续
    plot(cx(1:k/5-1),attmse(6,1:(length(attmse))),color(1,6),'LineWidth',type(6));
    hold on;
    xlabel('时间(s)');
    ylabel('AO');
    title('有攻击情况下编队节点的AO之和');
end

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

