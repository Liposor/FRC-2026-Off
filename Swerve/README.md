# Swerve 2026 - WPILib, Phoenix 6, PathPlanner e MapleSim

Base Java para o drivetrain do robo descrito pelo time. O projeto foi organizado para separar o
codigo diretamente ligado aos subsistemas das ferramentas reutilizaveis de swerve.

## Hardware conhecido

- Modulos SDS MK5n, reducao R3 confirmada: `5.27:1`.
- Rodas MK5n originais, diametro nominal de 4 polegadas.
- Kraken X60 no drive e Kraken X44 no steer.
- Um CANcoder por modulo e Pigeon 2.0.
- CANivore dedicado ao swerve.
- REV PDH; ID CAN provisoriamente `1`.
- Massa informada: `51.79 kg`.
- Baterias entre `12.3 V` e `12.4 V` no uso, ate `12.7 V` em excelente estado.
- Tres Limelight 4 com HALO: frente, esquerda e direita.

O R3 tem velocidade livre teorica de 19.2 ft/s, aproximadamente `5.852 m/s` com X60 FOC. O
controle do piloto esta inicialmente limitado a `4.5 m/s`; velocidade teorica nao e velocidade
segura garantida no carpete.

## Bloqueio do robo real

O programa permanece deliberadamente bloqueado no robo real. A simulacao usa placeholders, mas
`SwerveHardwareConfig.isReadyForRealHardware()` so libera o hardware depois de confirmar:

1. distancia longitudinal e lateral entre os centros dos modulos;
2. comprimento e largura externos dos bumpers;
3. IDs CAN dos oito motores, quatro CANcoders, Pigeon e PDH;
4. offset absoluto dos quatro CANcoders;
5. transform 3D de cada Limelight em relacao ao centro do robo;
6. limites de corrente e corrente de slip medidos;
7. ganhos de drive e steer validados por SysId e testes.
8. coupling ratio `54/16` confirmado no Tuner X.

Tambem precisamos confirmar o tipo exato da roda original instalada, o ID do PDH e se existe
licenca Phoenix Pro. Enquanto a licenca nao for confirmada, o codigo usa `RemoteCANcoder`;
`FusedCANcoder` so e selecionado por configuracao explicita.

## Arquitetura

```text
frc/robot/
  lib/swerve/
    characterization/  SysId e raio efetivo
    config/             hardware, visao, controle e localizacao
    control/            setpoints, X-lock e DriveToPose
    diagnostics/        CAN, PDH, motores, CANcoders, loop e memoria
    hardware/           constantes Phoenix/Tuner X
    localization/       historico, colisao, slip e recuperacao
    logging/            telemetria CTRE/NT/Hoot
    simulation/         MapleSim e injecao de falhas
    state/              RobotState central com historico fixo
    vision/             IO, confiabilidade, consenso e simulacao
  subsystems/
    swerve/              somente o subsystem do drivetrain
    vision/              fusao das tres Limelights
    superstructure/      goals e sequencias de alto nivel
```

## Precisao do swerve

`SwerveHardwareConfig` separa raio nominal e raio efetivo. A reducao mecanica continua em 5.27; a
correcao por desgaste/compressao deve alterar `EFFECTIVE_WHEEL_RADIUS_SCALE` somente depois da
caracterizacao no robo.

O teleop usa `SwerveSetpointGenerator` do PathPlanner. Ele recebe a tensao atual da bateria e limita
a transicao entre estados de modulo, evitando pedir instantaneamente aceleracoes e angulos
impossiveis. Se `RobotConfig` nao puder ser carregado, a protecao faz bypass explicito e publica
`Swerve/SetpointGeneratorEnabled=false`.

O modelo do PathPlanner ja usa massa de 51.79 kg, roda de 0.0508 m, R3 e velocidade teorica de
5.85216 m/s. MOI, geometria e tamanho do robo continuam provisiorios; portanto os feedforwards nao
devem ser considerados calibrados.

## Superstructure e score

Goals existentes:

- `IDLE`
- `COLLECTING`
- `HOLDING_GAME_PIECE`
- `ALIGNING_TO_SCORE`
- `READY_TO_SCORE`
- `SCORING`
- `SCORE_COMPLETE`
- `CLIMBING`

`Superstructure.alignAndScoreCommand(...)` executa o `DriveToPose`, exige erro pequeno de posicao e
heading, baixa velocidade e estabilidade por 200 ms. Somente depois entra em `READY_TO_SCORE` e
`SCORING`, quando o solver aplica X-lock. Assim o robo nao trava as rodas antes de terminar o
alinhamento. Ainda faltam as poses reais de score do jogo/mecanismo para ligar essa sequencia aos
botoes.

## Tres Limelights e MegaTag2

Nomes atuais:

- `limelight-front`
- `limelight-left`
- `limelight-right`

Cada frame passa por gates de timestamp, tag conhecida, distancia, ambiguidade, limites do campo,
altura Z, roll/pitch, giro do robo e inovacao contra a pose atual. Na convencao WPILib, altura e Z,
nao Y.

As cameras nao sao mais fundidas como tres sensores independentes no mesmo instante:

- duas ou tres cameras que concordam geram uma unica pose ponderada;
- uma camera plausivel e aceita com desvio-padrao aumentado;
- cameras simultaneas que discordam bloqueiam a fusao daquele ciclo;
- heading visual recebe incerteza muito alta; Pigeon/odometria continuam como fonte angular;
- heartbeat parado por mais de 500 ms marca a camera desconectada.

As transforms atuais sao apenas para simulacao. Medir em metros a partir do centro do robo:
`+X` frente, `+Y` esquerda, `+Z` cima, roll em X, pitch em Y e yaw em Z.

## Odometria, colisao e recuperacao

Um Pigeon nao mede velocidade translacional. Em 2.7 m/s constantes, aceleracao proxima de zero e
normal. O detector combina:

- velocidade pedida e medida pelas rodas;
- impacto e jerk do Pigeon;
- divergencia entre aceleracao inferida pelas rodas e aceleracao do IMU;
- bias lento de aceleracao aprendido somente quando o robo esta parado.

Uma confianca continua e publicada em `OdometryHealth/CollisionConfidence`. O estado so vira
`BLOCKED` depois de debounce; nesse estado o solver e o PathPlanner usam X-lock. O rollback
automatico permanece desligado porque outro robo pode realmente deslocar o chassi.

O historico circular salva pose e velocidades a cada 20 ms, sem crescer memoria. A pose anterior ao
impacto e reconstruida por integracao robot-relative e pode ser aplicada manualmente com `Back`,
desde que esteja a menos de 0.50 m da pose atual. `RobotState` mantem um segundo historico central
para diagnostico temporal e futura reproducao de eventos.

## Simulacao MapleSim

No desktop, o MapleSim substitui o integrador simples da Phoenix e injeta estados nos TalonFX,
CANcoders e Pigeon. O modelo usa:

- massa de 51.79 kg;
- Kraken X60 FOC e Kraken X44 FOC;
- R3, roda nominal de 4 polegadas e atrito provisoriamente 1.20;
- geometria dos centros dos modulos;
- bumpers provisoriamente 30 x 30 polegadas.

O JSON oficial do MapleSim consultado em agosto de 2026 referencia
`0.4.0-beta-obstacles-fix`, mas o Maven oficial nao publica esse artefato. O projeto fixa
`0.4.0-beta`, que e a ultima versao realmente listada no metadata oficial. Revise essa divergencia
antes de atualizar a dependencia.

Falhas podem ser injetadas por NetworkTables:

- `/SimulationFaults/VisionDropoutAll`
- `/SimulationFaults/VisionOutlierXMeters`
- `/SimulationFaults/ExtraVisionLatencyMilliseconds`
- `/SimulationFaults/CANcoderOffsetRotations`

A simulacao geometrica das Limelights calcula tags dentro do FOV e alcance e publica as mesmas
chaves consumidas pelo IO real. Ela nao modela pixels, reflexos, motion blur, exposicao, HALO,
oclusao por outro robo nem deformacao estrutural.

## Logs e saude

Tres camadas sao iniciadas:

- WPILib DataLog para Driver Station, joystick, comandos e NetworkTables;
- CTRE SignalLogger/Hoot para sinais Phoenix e SysId;
- topicos estruturados de pose, chassis e modulos para AdvantageScope.

Diagnosticos a 5 Hz registram corrente supply/stator, tensao, temperatura, erro de velocidade e
angulo, posicao absoluta e `MagnetHealth` de cada modulo, alem de tensao/corrente/temperatura do PDH.
Telemetria pesada continua limitada para proteger CPU, GC e CAN. O monitor observa atraso do loop,
temperatura da roboRIO, memoria Java, utilizacao, bus-off, TX-full, REC e TEC do barramento.

O CANivore e selecionado por `*` no robo real, adequado quando existe somente um CANivore. A
simulacao usa `rio`. Se outro CANivore for instalado, troque `*` pelo nome confirmado para evitar
selecionar o barramento errado.

## Caracterizacao

Os comandos aparecem em `Swerve/Characterization` e sao bloqueados fora de Test Mode:

- SysId de translacao;
- SysId de steer;
- SysId de rotacao;
- caracterizacao do raio efetivo por rotacao de oito segundos.

Execute em area aberta, sobre o piso de competicao, com bumpers, massa final e bateria observada.
Tenha uma pessoa no E-stop. O resultado do raio e apenas registrado; ele nao reescreve constantes
automaticamente.

Ordem recomendada no hardware:

1. validar IDs, inversoes e barramento no Phoenix Tuner X;
2. alinhar mecanicamente as rodas e gravar offsets dos CANcoders;
3. conferir `MagnetHealth` e saltos do absoluto por uma volta completa;
4. medir geometria dos modulos e bumpers;
5. validar Pigeon rigidamente montado, orientacao e bias;
6. caracterizar raio efetivo, drive, steer e rotacao;
7. medir slip current e queda de tensao;
8. ajustar setpoints e PathPlanner;
9. medir as tres cameras e validar cada uma isoladamente;
10. habilitar consenso e repetir trajetorias em varios pontos do campo.

## Executar

No terminal WPILib, dentro da pasta `Swerve`:

```powershell
.\gradlew.bat clean test
.\gradlew.bat simulateJava
```

Controles atuais:

- analogico esquerdo: translacao;
- analogico direito X: rotacao;
- `A`: X-lock manual;
- `B`: aponta os modulos;
- `LB`: redefine a frente field-centric;
- `Back`: aplica recuperacao historica recomendada;
- `RT`: coleta/segura peca simulada;
- `LT`: testa `READY_TO_SCORE` e X-lock;
- `Y`: climbing;
- `X`: idle;
- `RB`: simula posse de peca.

## Referencias

- [SDS MK5n](https://www.swervedrivespecialties.com/products/mk5n-swerve-module)
- [CTRE Phoenix 6 Swerve](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/mechanisms/swerve/swerve-overview.html)
- [Limelight MegaTag2](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2)
- [PathPlanner](https://pathplanner.dev/pplib-build-an-auto.html)
- [WPILib](https://docs.wpilib.org/en/stable/index.html)
- [MapleSim](https://github.com/Shenzhen-Robotics-Alliance/maple-sim)
- [Citrus Circuits 2026](https://github.com/frc1678/C2026-Public)
- [Citrus Circuits 2025](https://github.com/frc1678/C2025-Public)
- [Team 254 2025](https://github.com/Team254/FRC-2025-Public)
- [Team 2910 2025](https://github.com/FRCTeam2910/2025CompetitionRobot-Public)
- [Liposor/Odometry](https://github.com/Liposor/Odometry)

Os repositorios de equipes serviram como referencias de arquitetura: estado central e buffers de
tempo, readiness de alinhamento, telemetria por modulo e testes de visao. Ganhos e thresholds nao
foram copiados como se fossem universais; todos continuam sujeitos a validacao nos logs deste robo.
