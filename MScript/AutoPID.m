%%
clc
clear

%state = 'relay';
%state = 'Org';
state = 'PID';
%state = 'PI';

TypeControler = 1; % 1-Sem controle; 2-PID; 2-PID Anti-Windup
TypeOutput = 1; %  1-Sistema; 2-Degrau usado para simular; 3-Atraso de transporte 
T_PID = 1; % 1-PID; 2-PI+D
N = 1;
Malha = 1;

K=1;
Kp = 1;
Ti = 100;
T = 1/10;        
Td = 1;
tau = 1;
theta = 1;
h = 30;

switch state 
    % Conjunto de lógicas para se calcular o T e outras carcterísicas
    case 'pinfo' 
        data = SSimulink(1,1);
        PInfo(data);
        plot(data)
    
    % Estado par visualizar a planta original
    case 'Org'
        data = SSimulink(1,1);
        plot(data)
    
    % Cálculo e visualização do pid
    case 'PID'
        Malha = 2; %Abre a malha
        data = SSimulink(1,1);
        CTauTheta(data);
        CalcPID('PID',tau, theta);
        Malha = 1; %Fecha a Malha
        data = SSimulink(1,1);
        plot(SSimulink(2,1), data.Time(), data.Data())
        grid();
        axis([0 750 -inf inf])
        title('PID')
        xlabel('Tempo [s]')
        ylabel('Amplitude')
        legend('PID','Original');
        
    case 'relay'
        subplot(2,1,1);
        plot(SSimulink(4,1));
        grid();
        axis([0 750 -inf inf])
        title('Saida Relé')
        xlabel('Tempo[s]')
        ylabel('Amplitude')
        
        
        subplot(2,1,2); 
        axis([0 750 -inf inf])
        relay(h);
        plot(SSimulink(2,1));
        grid();
        axis([0 750 -inf inf])
        title('PID por Relé')
        xlabel('Tempo [s]')
        ylabel('Amplitude')
    case 'show'       
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

function [] = CTauTheta(data)

        len = length(data.Time());
        K = data.Data(len); % Valor em regime  permanente
        assignin('base', 'K', K);
        % Laço para buscar os valores de tempo da resposta transitória
        c_t1 = 0;
        for i=1:len
            if(data.Data(i)>=K*0.283 && c_t1==0)
                t1 = data.Time(i);
                c_t1 = 1;
            end
            if(data.Data(i)>=K*0.632)
                t2 = data.Time(i);
                break
            end
        end

        A = [1 1/3; 1 1];
        B = [t1 ; t2];
        C= linsolve(A,B);
        theta = C(1,1);
        tau = C(2,1);
        assignin('base', 'tau', tau);
        assignin('base', 'theta', theta);
end

% Função para o cálculo dos parâmetros dos diferentes controladores
function CalcPID(type_controler, tau, theta)
    
    rtt = theta/tau;
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
            if rtt> 0.1 && rtt <= 0.4
                Kp = 1.2*tau/theta;
                Ti = 2*theta;
                Td = 0.5*theta;
            end
            if rtt >0.4 && rtt < 4.5
                Kp = 1.35*rtt^-1 + 0.27;
                Ti = 2.5*theta*(1+rtt/5)/(1+0.6*rtt);
                Td = 0.37*theta/(1+0.2*rtt);
            end  
    end
    assignin('base', 'Kp', Kp);
    assignin('base', 'Ti', Ti);
    assignin('base', 'Td', Td);
end

function PInfo(data)
    max = data.Data(1);
    Mp = 0;
    len = length(data.Time());

    
    for i=1:len-1
        if (data.Data(i)>max)
            max = data.Data(i);
            Mp = (data.Data(i)-100)/100;
        end
    end
    ts = 240; % Obtido de forma visual
    xi = sqrt(((log(Mp)/pi)^2)/(1+(log(Mp)/pi)^2));
    wn = 4/(ts*xi);
    T = 2*pi/(10*wn);
    
    assignin('base', 'T', T);
end

function relay(h)
    a = 137-90;
    Kc = 4*h/(pi*a);
    Tc = 1/2.99e-3;
    Kp = 0.6*Kc;
    Ti = Tc*0.5;
    Td = Tc*0.125;
    
    assignin('base', 'Kp', Kp);
    assignin('base', 'Ti', Ti);
    assignin('base', 'Td', Td);
end



