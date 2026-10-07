# Stardust 9200 — Dashboard Python / roboRIO 2026

Aplicativo desktop local em Python + PySide6, preto e roxo, NT4 (pyntcore 2026.2.2). Executar no computador da Driver Station, conectado à rede do robô. Não depende de navegador, internet ou servidor web.

## Instalar e abrir (Windows / Python 3.12)

Abra o terminal nesta pasta:

```powershell
py -3.12 -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
.venv\Scripts\python main.py
```

Para experimentar sem robô:

```powershell
.venv\Scripts\python main.py --demo
```

Linux: `python3 -m venv .venv`, `.venv/bin/python -m pip install -r requirements.txt`, `.venv/bin/python main.py`.

Selecione alvo A–L, nível L1–L4, marque **Armar controles**, e pressione uma ação. Seleções são locais até a ação. Os controles exigem conexão e o sinal `enabled` publicado pelo robô; Cancelar pode ser solicitado com esse sinal falso. Perder conexão desarma o painel. O dashboard não habilita o robô e não substitui a Driver Station. Cancelar é uma solicitação de software, não um emergency stop.

## Conexão e configuração

`config.json` contém equipe, servidor e tópicos. Por padrão usa `setServerTeam(9200)` e a descoberta da Driver Station no mesmo PC. Para IP fixo, configure `server` como `10.92.0.2`; USB pode usar `172.22.11.2`. Para simulação local, `127.0.0.1`. NT4 usa porta 5810. Altere `actions` para os identificadores de ações aceitos pelo seu robô. Leia o README antes de armar os controles reais.

## Contrato de integração — necessário no código do robô

**O projeto atual do repositório não possui um receptor desse protocolo. Instalar este dashboard sozinho não cria comandos no robô.** Nenhum arquivo Java foi alterado. O painel conecta ao servidor NT4 existente da roboRIO, mas ações só executam quando o programa do robô interpreta o contrato abaixo.

Dashboard publica string JSON em `/Dashboard9200/request`:

```json
{"id":"identificador-unico","session":"sessao-do-dashboard","action":"position","target":"A","level":"L2","issued_at_ms":1791400000000,"ttl_ms":1000}
```

Uma mensagem contém todos os argumentos, evitando leituras parciais entre tópicos. `id` muda em cada clique, inclusive para ações repetidas. Ações padrão: `position`, `intake`, `stow`, `cancel`.

O receptor deve processar somente eventos novos, ignorar o valor inicial/antigo ao se conectar, deduplicar `id`, validar argumentos e condições do robô, e agendar/cancelar seus comandos pelo scheduler. `ttl_ms` é a janela de validade desejada; `issued_at_ms` é horário Unix do PC e só pode ser comparado com o relógio do robô se sincronizados. Sem sincronização, use a idade do evento no relógio monotônico do receptor e nunca reproduza valores antigos. Mantenha os bloqueios normais da WPILib quando disabled.

Robô publica string JSON em `/Dashboard9200/ack`:

```json
{"id":"mesmo-identificador","status":"accepted","message":"Comando agendado"}
```

Use `rejected` para rejeição. `accepted` significa aceitação/agendamento, não conclusão física. O painel mostra apenas a resposta recebida. Sem resposta por 2 s, mostra “Sem confirmação”; não retransmite. Uma solicitação fica pendente por vez; Cancelar pode ser solicitado depois da resposta ou timeout. Nenhuma solicitação é enviada offline.

Telemetria que o robô deve publicar (tópicos também configuráveis):

| Tópico | Tipo | Conteúdo |
| --- | --- | --- |
| `/Dashboard9200/robot/enabled` | boolean | Estado habilitado real |
| `/Dashboard9200/robot/voltage` | double | Tensão da bateria em V |
| `/Dashboard9200/robot/mode` | string | TELEOP, AUTO, TEST ou DISABLED |
| `/Dashboard9200/robot/alliance` | string | AZUL, VERMELHA ou desconhecida |
| `/Dashboard9200/robot/state` | string | Estado atual da superestrutura |

Tópicos ausentes mostram “—” e `enabled` ausente mantém ações bloqueadas. NT conectado indica apenas transporte conectado. O programa não afirma execução física sem retorno do robô.

## Validar e capturar

```powershell
.venv\Scripts\python -m unittest discover -s tests -v
.venv\Scripts\python main.py --demo --screenshot preview.png
```

`preview.png` é captura da própria janela Qt com telemetria simulada, não de conexão real à roboRIO. Verificações: servidor NT4 local, envio JSON, confirmação por ID, repetição com IDs distintos, timeout, bloqueio offline e controles da interface. Hardware físico não foi testado.

API NT4: https://robotpy.readthedocs.io/projects/robotpy/en/2025.2.1/ntcore/NetworkTableInstance.html (mesmos métodos de cliente NT4 usados pela versão 2026 instalada).
