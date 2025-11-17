function [Lam0_opt,sol] = BangBangIdPMP
%indirect Pontryagin Maximium Principle, to get Bang-Bang optimal control
% Edited by Siyang Meng in Northwestern Polytechnical University
% 2021-10-9 17:28:11

% callfunction
% editor: Siyang Meng, from Northwestern Polytechnical University
amin = -2;
amax = 1;
var.R =0.3;
t0 = 0;
tf = 30;% 这是一个很大的值，肯定用不了这么长时间，因为tf是触发得到的，因此需要给足
yinit = [0;0];
yfinal= [100;0];

%% nonlinear root finding
events = @(t,x)switchingOff(t,x,var);
refine = 1;
% Hamilton ODE dynamical system integration options
options = odeset('Events',events,'OutputFcn',[],'refine',refine);
options.AbsTol=1e-6;
options.RelTol=1e-4;
options.MaxStep=0.01;
options.step=0.001;
% solve shooting problem
% levenberg-marquardt trust-region-reflective trust-region-dogleg
soloptions = optimoptions('fsolve','Display','off','Algorithm','levenberg-marquardt',...
    'TolX',1e-8,'FunctionTolerance',1e-6,'MaxFunEvals',1000);
Lam00 = [-0.5 ; -3];
[rt,dXf,tout,yout,uout,teout,yeout,ieout] = ShootingTmlErr(Lam00,1);
plotUXL(tout,yout,uout);
[Lam0_opt,FVAL,EXITFLAG,OUTPUT,JACOB] = fsolve(@ShootingTmlErr,Lam00,soloptions);
%% result
[rt,dXf,tout,yout,uout,teout,yeout,ieout] = ShootingTmlErr(Lam0_opt,1);
sol = struct;
sol.var = var;
sol.xopt = Lam0_opt;
sol.rt = rt;
sol.t = tout;
sol.x = yout;
sol.u = uout;
sol.teout = teout;
sol.yeout = yeout;

% plot result
plotUXL(tout,yout,uout);
%%
    function plotUXL(t,yout,uout)
        t=tout;    y=yout;
        figure(1)
        subplot(2,1,1)
        plot(t,y(:,1),'k-'),title('state'),ylabel('x/m');
        subplot(2,1,2)
        plot(t,y(:,2),'k-'),xlabel('t/s'),ylabel('v/m/s');
        hold off

        figure(2)
        plot(t,uout);xlabel('t/s'),ylabel('accleration'),title('control');
        a=diff(y(:,2))./diff(t);
        hold on
        plot(t(1:end-1),a)
        hold off

        figure(3)
        plot(t,y(:,3:4));xlabel('t/s'),ylabel('costate');
        legend('\lambda_x','\lambda_v')
        hold off
    end

    function dx = idHamiltonSystem(t,x,var,uflag)
        % a template for Bang-Bang control, Bang-Bang control solved in segments
        state = x(1:2);
        costate = x(3:4);
        switch uflag
            case 'min'
                u= amin;
            case 'max'
                u= amax;
            case 'sf'
                u = - x(4)/var.R;
        end
        dstate = [state(2);
            u];
        dcostate = [0 ;
            -costate(1)];
        dx = [dstate;
            dcostate];
    end

    function [value,isterminal,direction,uflag] =switchingOff(t,x,var)
        % a template for Bang-Bang control, to judge when to switch
        Sf = - x(4)/var.R;
        if Sf<amin
            uflag = 'min';
        else if Sf>amax
                uflag = 'max';
            else
                uflag = 'sf';
            end
        end
        value = [Sf-amin;
                 Sf-amax;
                 x(2)];     % Detect v = 0
        isterminal = [1;1;1];    % Stop the integration
        direction = [0;0;-1];    % direction
    end
    function [rt,dXf,tout,yout,uout,teout,yeout,ieout] = ShootingTmlErr(Lam0,outputSave)
        % nonlinear root finding problem, OCP shooting formulate
        % input: costate initial guess
        % output: terminal constraint error,
        %         switching Hamiltonian system: swtiching points, state and
        %         costate
        Lam0 = reshape(Lam0,2,1);
        x0 = [yinit;
            Lam0];
        
        % 数值积分求解系统
        tstart = t0;
        y0 = x0;     % 每一段迭代更新
        [~,~,~,onOff] =switchingOff(t0,y0,var);
        if nargin==2&&outputSave
            tout = tstart;
            yout = x0';
            teout = [];
            yeout = [];
            ieout = [];
        end
        
        for i = 1:100
            % Solve until the first terminal event.
            system = @(t,x)idHamiltonSystem(t,x,var,onOff);
            [t,y,te,ye,ie] = ode45(system,[tstart tf],y0,options);
            
            % Accumulate output.  This could be passed out as output arguments.
            nt = length(t);
            if nargin==2&&outputSave
                tout = [tout; t(2:nt)];
                yout = [yout; y(2:nt,:)];
                teout = [teout; te];          % Events at tstart are never reported.
                yeout = [yeout; ye];
                ieout = [ieout; ie];
            end
            tstart = t(nt);
            if sum(ie==3)||tstart==tf
                if nargin==2&&outputSave
                    teout = [teout; tf];          % Events at tstart are never reported.
                    yeout = [yeout; yout(end,:)];
                    ieout = [ieout; 0];
                end
                break;
            else
                y0 = reshape(ye(end,:),4,1);
                % Set the new control throttle
                [~,~,~,onOff] =switchingOff(te,ye,var);
            end
        end
        if nargin==1
            yout=y;
        end
        uout = [- yout(:,4)/var.R];
        uout(uout>amax)=amax;
        uout(uout<amin)=amin;
        
        % terminal state triggered
        y_end =  y(end,:)';
        a = uout(end);
        Hf= 1+y_end(4)*a+0.5*var.R*a^2;
        dXf = [yfinal - y_end(1:2)];
%         rt = dXf;
        rt = [dXf(1);
             Hf-0];
    end
end
