# STORM 9200 · FRC 2026 Off-season

Projeto WPILib 2026.2.1 em Java 17. Abra esta pasta no VS Code do WPILib. O número de equipe é 9200.

```powershell
.\gradlew.bat build
.\gradlew.bat simulateJava
.\gradlew.bat deploy
```

O deploy instala o programa, a tela, o layout Elastic e os arquivos do PathPlanner. A tela fica em `http://10.92.0.2:5800/` no robô e `http://localhost:5800/` na simulação. O ping exibido é o tempo de ida e volta da consulta HTTP. Para abrir em tela cheia no Windows, use F11.

## Controles

| Controle | Ação |
| --- | --- |
| Piloto · sticks | Translação e giro. Um movimento forte toma o controle de uma rota ou mira. |
| Piloto · RT | Coleta com translação limitada a 1,8 m/s. |
| Piloto · LT | Tiro no próprio HUB. Fora da zona, vai ao ponto permitido mais próximo dos quatro e então mira. Solte para interromper o pedido. |
| Piloto · A / B | A trava as rodas em X. B cancela assistência e pedidos da tela. |
| Piloto · LB / LS | LB redefine a frente para o piloto. LS alterna modo lento. |
| Piloto · RS / Start + RS | RS corrige o heading com MT1. Com Start, exige confirmação de posição pelo MT2 mesmo com várias tags. Faça parado, com tags visíveis. |
| Piloto · Y | Alterna a correção gradual de heading por visão. O botão junto ao mapa mostra o mesmo estado. |
| Piloto · Back | Aplica a pose histórica recomendada, quando disponível. |
| Piloto · POV direita / esquerda | Inicia/cancela a calibração de geometria do swerve em Test. |
| Controle 2 · A | Executa o estado selecionado no Elastic enquanto pressionado. |
| Controle 2 · B | OUTTAKE. Inverte coleta e os três rolos e oscila a mira em até 5° antes de manter a direção de início. |
| Controle 2 · X | Reduz em 150 RPM o alvo do mapa enquanto pressionado. |
| Controle 2 · LB | Autoriza pedidos da tela enquanto pressionado. |
| Controle 2 · RB | Autoriza o giro do shooter de bancada com calibração ativa e teste armado pela tela. |

## Tela e Elastic

- Na tela, toque no mapa e confirme em **Ir ao destino**. Os quatro botões chamam poses de tiro e as duas rotas chamam trajetos PathPlanner com coleta. Soltar LB interrompe os pedidos da tela. A rotação do destino livre mantém o heading do início do pedido.
- O cabeçalho mostra o tempo restante do período, o tempo restante do shift e o estado do HUB da aliança. No REBUILT são 20 s de AUTO e 2:20 de TELEOP, com 10 s de transição, quatro shifts de 25 s e 30 s de Endgame. O console usa o relógio da Driver Station. Fora do FMS, se o tempo estiver indisponível, usa um cronômetro identificado como treino. Se faltar Game Data nos shifts 1–4, o HUB aparece como indefinido. Esses indicadores são informativos.
- Abra **Calibração** desabilitado ou em Test, fora do FMS. O mapa dá lugar aos ajustes de RPM, flap, intake e rolos. Os valores são ajustes em RAM. Para tornar permanentes os valores validados, incorpore-os em `ConfigMechanisms.java`.
- Antes do primeiro uso do shooter, coloque fisicamente o flap no zero mecânico com o robô desabilitado. Confirme o zero na tela. Repita após desligar ou reinicializar o controlador. O teste armado gira o shooter com o swerve parado e em X. A alimentação de bolas fica bloqueada durante esse teste.
- No Elastic 2026, use **File → Import Layout** e selecione `src/main/deploy/elastic-layout.json`. O arquivo segue o esquema do Elastic 2026.1.1 e também pode ser baixado em `http://10.92.0.2:5800/elastic-layout.json`. Conecte à equipe 9200 e use as abas **Operação**, **Shooter**, **Mecanismos**, **Swerve** e **Rede e visão**.
- No AdvantageScope 2026, conecte ao robô. Use `/Swerve/Pose` na aba **2D Field** e `/Swerve/ModuleStates` e `/Swerve/ModuleTargets` na aba **Swerve**. Selecione módulos na ordem **FL, FR, BL, BR**. A pose usa origem azul em ambas as alianças.

## Hardware e lógica

| Componente | Configuração |
| --- | --- |
| Swerve | Phoenix 6. Mantém `generated/TunerConstants.java` da pasta local, com os offsets dos quatro módulos, CANivore, Pigeon e odometria a 250 Hz. |
| Intake | Kraken 32 no roboRIO. Coleta a 85 RPS (5.100 RPM) com teto de 90 RPS na calibração. DIO 0 reporta o encoder absoluto. O controlador 31 do pivô está ausente do código. |
| Rolos | Kraken 21 inferior, 22 e 23 superiores. O 23 tem inversão de Houston. Rolos superiores têm malha de velocidade em RPS. |
| Shooter | Kraken 24 e 25, com inversão no 25. Malha de velocidade usa volts, `kV=0,13 V/RPS` e o mapa em RPM de Houston. |
| Flap | NEO Vortex / Spark Flex 61. Redução 10:1 e curso 0–40°. Ganho inicial 0,04, limite de saída ±18% e corrente 15 A. |

O fluxo de tiro exige pose inicializada, aliança conhecida, margem dentro da zona, mira no HUB, velocidade medida abaixo de 0,10 m/s, rodas confirmadas em X e feedback do shooter e dos rolos. A alimentação começa após 100 ms de prontidão. Se a roda perde RPM, os rolos param enquanto o controlador tenta recuperar o mesmo alvo. Se a bateria limitar a tensão disponível, a cadência cai pela espera de recuperação. O alvo de RPM permanece fixo. O gatilho de proteção de potência usa 7 V e o indicador de brownout.

Falhas e alertas de CAN aparecem por dispositivo. Uma leitura ausente no shooter ou nos rolos suspende a alimentação e permite a condução manual. Sem odometria recente, o controle passa para robot-centric limitado e a navegação automática termina. As falhas CAN retidas são indicadores de diagnóstico. O MatchAnalyser usa corrente e quedas breves de velocidade na coleta e no tiro. Os contadores representam eventos e o estado de bolas é uma estimativa.

Os sinais de controle usam leitura não bloqueante e o diagnóstico é escalonado. O Elastic mostra os máximos recentes de duração do scheduler e da superestrutura. Use esses canais para comparar com o robô carregado. O build e os testes desktop não medem CommandSchedulerOverrun no roboRIO.

## Campo e Limelight

- A grade de navegação local recebeu uma célula cardinal de 0,30 m a mais de folga para cobrir os 0,16 m do intake na frente. As dimensões do PathPlanner usam 0,922 × 0,762 m e deslocamento frontal de 0,08 m. A grade bloqueia passagens estreitas que precisam ser validadas no robô.
- As quatro poses azuis são `x = 2,75 m`, com `y = 1,50 / 2,65 / 5,40 / 6,55 m`. São espelhadas por rotação de 180° para a aliança vermelha. As duas rotas de coleta foram escolhidas na zona neutra a partir da grade local. Confirme o ponto inicial, a folga em curvas e a altura do intake antes de usar em campo.
- A pasta local configura a câmera `limelight-front`. Câmeras adicionais entram em `ConfigVision.CAMERAS` quando o nome, a montagem e a calibração estiverem confirmados. O ajuste de −0,14 m foi interpretado no eixo longitudinal do robô, para trás, aplicado a X/Y em MT1 e MT2. Valide o sinal com o robô a 0° e 180°.
- O MT2 recebe heading e taxas continuamente, com yaw do Pigeon corrigido no referencial do campo. O reset com confirmação de MT2 compara posição, idade e IDs com o MT1. O ângulo absoluto vem do MT1, pois o MT2 usa o próprio yaw do robô como entrada. Depois do reset, segure o robô parado até o painel mostrar **Pose referenciada**.
- **Correção de heading por visão** começa desligada a cada reboot. Em teleop, compara o yaw de pelo menos duas tags do MT1 com o yaw bruto interpolado do Pigeon no instante da captura, convertido para a referência de campo do Phoenix. Avalia até 5 vezes por segundo, aceita tags a até 4,5 m e giro a até 45°/s, e aplica até 0,6° por avaliação. Resíduos acima de 20° ficam para o reset manual. Quando falta visão ou a função é desligada, a última correção continua na referência do swerve. Os resets manuais e de pose invalidam os frames anteriores e zeram o acumulado mostrado no painel. As constantes estão em `ConfigVision` e os diagnósticos em `/SmartDashboard/Vision/HeadingAssist`.
- O autônomo padrão é **Parado**. As rotas fornecidas são assistências de teleop. O projeto aceita novos arquivos `.auto` do PathPlanner na pasta de deploy.

## Referências preservadas

- [STORM-2026](https://github.com/Liposor/STORM-2026) · base Houston para IDs, inversões, geometria do flap e mapa de tiro. Arquivos conferidos em `DeathStar/src/main/java/frc/robot/Subsystems`.
- [Liposor/Swerve](https://github.com/Liposor/Swerve) · arquitetura do drivetrain, simulação, filtros e referência de ângulo. A pasta local `C:/DEATH STAR/STORM - Apenas Swerve` tem prioridade para calibração e IDs físicos.
- [FRC-2026-Off](https://github.com/Liposor/FRC-2026-Off) · biblioteca de off-season. Revisado o RobotContainer da pasta Swerve.
- [Team 2910 · SuperStructure](https://github.com/FRCTeam2910/2026CompetitionRobot-Public/blob/main/src/main/java/org/frc2910/robot/subsystems/SuperStructure.java) e [ShooterSubsystem](https://github.com/FRCTeam2910/2026CompetitionRobot-Public/blob/main/src/main/java/org/frc2910/robot/subsystems/shooter/ShooterSubsystem.java) · referência para pedido do operador, estado em execução e liberação do hopper por prontidão. Hardware e implementação do STORM são próprios do projeto.
- [FIRST · Manual REBUILT](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf) · dimensão da zona, G407 e porta 5800 da faixa de equipe.
- [WPILib · Game Data do REBUILT](https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html) · leitura do relógio e identificação do primeiro HUB inativo.
- [Mapa REBUILT usado pelo Elastic](https://www.chiefdelphi.com/t/2026-rebuilt-high-resolution-field-image/510611) · recorte da imagem de campo sem bolas distribuída na instalação Elastic 2026.
- [Stardust 9200](https://www.frc9200.com/) · referência visual e logotipo do console. A fonte Archivo Black está incluída localmente com sua licença OFL.
- [PathPlanner · grade](https://pathplanner.dev/gui-navigation-grid-editor.html) e [Limelight · MegaTag2](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2) · convenções usadas no projeto.
- [Orbit 1690 · Kepler](https://www.chiefdelphi.com/t/orbit-1690-2026-robot-reveal-kepler/515277?page=5) · relato de deriva do Pigeon2. [Phoenix 6 · resetRotation](https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/swerve/SwerveDrivetrain.html#resetRotation(edu.wpi.first.math.geometry.Rotation2d)) · ponto de aplicação da correção de ângulo no STORM.

## Validação realizada

- Build Gradle WPILib 2026.2.1 concluído com sete testes de visão.
- A aplicação iniciou em simulação desktop. O Edge em 1024 × 600 carregou as duas rotas e executou abertura da calibração, ajuste de RPM e volta à arena.
- As oito poses de tiro e os extremos das rotas de coleta passaram pela checagem de conectividade da grade. O JSON do Elastic foi conferido com o esquema da versão instalada.
- O robô físico ainda precisa da conferência do sentido dos motores, do zero e curso do flap, do sinal do offset e da passagem com intake estendido. A comparação de tempo de loop e cadência de tiro precisa de logs no roboRIO.

