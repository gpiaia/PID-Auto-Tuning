%%
clc
clear

state = 'PID';
TypeControler = 1; % 1-Sem controle; 2-PID; 2-PID Anti-Windup
TypeOutput = 1; %  1-Sistema; 2-Degrau usado para simular; 3-Atraso de transporte 

K=1;
Kp = 1;
Ti = 100;
T = 1/10;        
Td = 1;
tau = 1;
theta = 1;

switch state
    
    case 'PID'
        data = SSimulink(1,1);
        %CTauTheta(data.Time(), FilterData(data));
        %plot(SSimulink(1,3));
        CalcPID(tau, theta);
    case 'pid'
    case 'tunnunig'
    case 'show'       
end

% Filtro do sinal para possibilitar os cálculos de tau e theta
function [avg] = FilterData(data)
        wSize = 15; 
        b = (1/wSize)*ones(1,wSize);
        a = 1;
        avg = filter(b, 1, data().Data());
end

% Função para simular no simulink e visualizar as saídas
function [outputs] = SSimulink(a, b) 
        assignin('base', 'TypeControler', a);
        assignin('base', 'TypeOutput', b);
        simOut = sim('TGB_V0000','SimulationMode','normal','AbsTol','1e-5',...
            'SaveState','on','StateSaveName','xout',...
            'SaveOutput','on','OutputSaveName','simout',...
            'SaveFormat', 'Dataset');
        outputs = simOut.get('simout');
end

function [] = CTauTheta(time, data)

        len = length(time);
        K = data(len); % Valor em regime  permanente
        assignin('base', 'K', K);
        % Laço para buscar os valores de tempo da resposta transitória
        c_t1 = 0;
        for i=1:len
            if(data(i)>=K*0.283 && c_t1==0)
                t1 = time(i);
                c_t1 = 1;
            end
            if(data(i)>=K*0.632)
                t2 = time(i);
                break
            end
        end

        A = [1 1/3; 1 1];
        B = [t1 ; t2];
        C= linsolve(A,B);
        tau = C(1,1);
        theta = C(2,1);
        assignin('base', 'tau', tau);
        assignin('base', 'theta', theta);
end

% Função para o cálculo dos parâmetros dos diferentes controladores
function CalcPID(type_controler, tau, theta)
    rtt = theta/tau; % Relação entre tau e theta
    
    switch type_controler
        case 'P'
            if rtt> 0.1 && rtt < 0.4
                Kp = rtt^-1;
                Ti = inf;
                Td = 0;
            end
            if rtt >0.6 && rtt < 4.5
                Kp = rtt^-1 + 1/3;
                Ti = inf;
                Td = 0;
            end
            
        case 'PI'
            if rtt> 0.1 && rtt < 0.4
                Kp = 0.9*rtt^-1;
                Ti = 3.33*tau*rtt;
                Td = 0;
            end
            if rtt >0.6 && rtt < 4.5
                Kp = 0.9*rtt^-1 + 0.082;
                Ti = 3.33*tau*rtt*(1+rtt/11)/(1+2.2*rtt);
                Td = 0;
            end            
        case 'PID'
            if rtt> 0.1 && rtt < 0.4
                Kp = 0.9*rtt^-1;
                Ti = 3.33*tau*rtt;
                Td = 0;
            end
            if rtt >0.6 && rtt < 4.5
                Kp = 0.9*rtt^-1 + 0.082;
                Ti = 3.33*tau*rtt*(1+rtt/11)/(1+2.2*rtt);
                Td = 0;
            end  
    end
    assignin('base', 'Kp', Kp);
    assignin('base', 'Ti', Ti);
    assignin('base', 'Td', Td);
end