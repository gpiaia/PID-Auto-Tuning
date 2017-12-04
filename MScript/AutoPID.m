%%
clc
clear

TypeController = 1; % 1-Sem controle; 2-PID; 2-PID Anti-Windup
TypeOutput = 1; %  1-Sistema; 2-Degrau usado para simular; 3-Atraso de transporte 
N = 1;
Malha = 1; % Controlar se a malha fica aberta ou fechada
Dev = 1;
CIPD = 1;
AntiW = 1;

%Definição de variáveis globais ao Simulink e mcode
K=1;
Kp = 1;
Ti = 100;
T = 1/10; % Menor do que o valor calculado pela função PInfo    
Td = 1;
tau = 1;
theta = 1;
h = 30;

% Conjunto de lógicas para se calcular o T e outras carcterísicas
% data = SSimulink(1,1);
% PInfo(data);

% Prompt para executar a opção desejada de controle
prompt = 'Digite 1->Sistema Original, 2->PIDs controladores, 3->relé, 4->Atraso de Transporte:';
state = input(prompt);

switch state     
    % Estado par visualizar a planta original
    case 1
        data = SSimulink(1,1);
        plot(data);
        grid();
        axis([0 350 -inf inf])
        title('Resposta ao degrau Original')
        xlabel('Tempo [s]')
        ylabel('Amplitude')
        legend('Original');
    
    % Cálculo e visualização do PID
    case 2
        prompt = 'Digite 1-> PID, 2->PD+I, 3->I-PD:';
        state2 = input(prompt);

        Malha = 2; %Abre a malha
        data = SSimulink(1,1);
        CTauTheta(data);
        CalcPID(tau, theta);
        Malha = 1; %Fecha a Malha
        data = SSimulink(1,1);
        type = 'PID';
        if state2==2
            Dev = 2; %Habilita o derivativo direto da realimentação
            type = 'PI+D';
        end
        if state2 == 3
            CIPD = 2;
            Dev = 2;
            type = 'I-PD';
        end
        
        pid = SSimulink(2,1);
 
        Dev = 2; %Habilita o derivativo direto da realimentação
        AntiW = 2; %Habilita o Anti-Windup
        anti = SSimulink(2,1);
        plot(pid.Time(),pid.Data(), data.Time(), data.Data(), anti.Time(), anti.Data());
        grid();
        axis([0 350 -inf inf])
        title('PID')
        xlabel('Tempo [s]')
        ylabel('Amplitude')
        legend(type,'Original', 'Anti-Windup');
        
    %Cálculo e visualização do PID por relé
    case 3
        subplot(2,1,1);
        plot(SSimulink(3,1));
        grid();
        axis([0 350 -inf inf])
        title('Saíca com o uso do Relé')
        xlabel('Tempo[s]')
        ylabel('Amplitude')
        
        subplot(2,1,2); 
        axis([0 350 -inf inf])
        relay(h); %Cálculo dos parâmetros do Controlador
        plot(SSimulink(2,1));
        grid();
        axis([0 350 -inf inf])
        title('PID pelo método do Relé')
        xlabel('Tempo [s]')
        ylabel('Amplitude')
    
    %Atraso de transporte
    case 4
        data = SSimulink(1,1);
        CTauTheta(data);
        plot(SSimulink(1,3), data.Time(), data.Data())
        grid();
        axis([0 350 -inf inf])
        title('Atraso de Transporte')
        xlabel('Tempo [s]')
        ylabel('Amplitude')
        legend('Atraso','Original');
end

% Função para simular no simulink e visualizar as saídas
function [outputs] = SSimulink(a, b) 
        assignin('base', 'TypeController', a);
        assignin('base', 'TypeOutput', b);
        simOut = sim('TGB_V0001','SimulationMode','normal','AbsTol','1e-5',...
            'SaveState','on','StateSaveName','xout',...
            'SaveOutput','on','OutputSaveName','simout',...
            'SaveFormat', 'Dataset');
        outputs = simOut.get('simout');
end

function [] = CTauTheta(data)
        len = length(data.Time());
        K = data.Data(len)/100; % Valor em regime  permanente
        assignin('base', 'K', K);
        
        % Laço para buscar os valores de tempo da resposta transitória
        c_t1 = 0;
        for i=1:len
            if(data.Data(i)>=100*K*0.283 && c_t1==0)
                t1 = data.Time(i);
                c_t1 = 1;
            end
            if(data.Data(i)>=100*K*0.632)
                t2 = data.Time(i);
                break
            end
        end

        %Solução do sistema linear para encontrar os valores de theta e tau
        tau = (3/2)*(t2-t1);
        theta = t2-tau;
        
        assignin('base', 'tau', tau);
        assignin('base', 'theta', theta);
end

% Função para o cálculo dos parâmetros dos diferentes controladores
function CalcPID(tau, theta)
    rtt = theta/tau;
    %Z&N
    if rtt> 0.1 && rtt <= 0.4 
        Kp = 1.2*tau/theta;
        Ti = 2*theta;
        Td = 0.5*theta;
    end
    %CC
    if rtt >0.4 && rtt < 4.5
        Kp = 1.35*rtt^-1 + 0.27;
        Ti = 2.5*theta*(1+rtt/5)/(1+0.6*rtt);
        Td = 0.37*theta/(1+0.2*rtt);
    end  
    assignin('base', 'Kp', Kp);
    assignin('base', 'Ti', Ti);
    assignin('base', 'Td', Td);
end

function PInfo(data)
    max = data.Data(1);
    Mp = 0;
    len = length(data.Time());

    % Busca por Mp
    for i=1:len-1
        if (data.Data(i)>max)
            max = data.Data(i);
            Mp = (data.Data(i)-100)/100;
        end
    end
 
    ts = 240; % Obtido de forma visual
    xi = sqrt(((log(Mp)/pi)^2)/(1+(log(Mp)/pi)^2));
    wn = 4/(ts*xi);
    T = (100*wn)/2*pi;
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