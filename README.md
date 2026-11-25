# ACE_MINIPROJ — seguidor de linha com encoders e FreeRTOS

Robô seguidor de linha para a Raspberry Pi Pico W (placa Pico4Drive). Ele junta duas coisas:

- **FreeRTOS** do projeto FreeRTOS-MicroRato, para dividir o trabalho entre os dois núcleos.
- **Controle de velocidade por encoder** (PID + feedforward) do LAB1.

## Instalação

O FreeRTOS **já vem dentro** do core arduino-pico (earlephilhower), na versão SMP para os dois núcleos. Não é preciso instalar biblioteca, clonar o pico-sdk nem configurar CMake. Basta:

1. Ter o VS Code com a extensão **PlatformIO IDE**.
2. Abrir esta pasta no PlatformIO e compilar (✓). No primeiro build, o PlatformIO baixa sozinho a plataforma `maxgerhardt/platform-raspberrypi`, o core arduino-pico e o toolchain. Isso leva alguns minutos e precisa de internet.
3. Gravar (→) com a Pico conectada. Se ela não for detectada, segure BOOTSEL ao ligar o cabo.

O que liga o FreeRTOS está no `platformio.ini`:

```ini
platform = https://github.com/maxgerhardt/platform-raspberrypi.git   ; NÃO a "raspberrypi" oficial (usa Mbed, sem FreeRTOS)
board_build.core = earlephilhower
build_flags = -DPIO_FRAMEWORK_ARDUINO_ENABLE_FREERTOS
```

No código, basta `#include <FreeRTOS.h>` e `#include <task.h>`. Toda a API do FreeRTOS fica disponível (`xTaskCreate`, `vTaskDelayUntil`, filas, semáforos...).

## Organização

| Arquivo | Conteúdo |
|---|---|
| `include/config.h` | **todos os parâmetros ajustáveis** (pinos, períodos, ganhos, geometria, sensores) |
| `src/main.cpp` | tarefas FreeRTOS, seguidor de linha, comandos (USB e WiFi), botões |
| `src/hw.cpp/.h` | drivers: encoders, motores, bateria, sensores IR |
| `src/PID.cpp/.h` | PID com feedforward e anti-windup |
| `src/commands.cpp/.h` | leitor de comandos seriais (do LAB1) |

## Arquitetura

| Núcleo | O quê | Período | Prioridade |
|---|---|---|---|
| 1 | interrupção dos encoders (timer) | 25 µs (40 kHz) | — |
| 1 | `control_task`: seguidor → v, w → rpm → PID+Kf → motores | 20 ms | 4 |
| 1 | `sensor_task`: lê os 5 sensores IR pelo MUX | 5 ms | 3 |
| 0 | `loop()`: comandos (USB e WiFi), botões, telemetria | ~5 ms | 1 |
| 0 | WiFi/LWIP (do core arduino-pico) | — | — |

- **Encoders sem trava.** Os contadores são cumulativos e só a interrupção escreve neles. Quem lê calcula a diferença, então não é preciso desligar interrupções entre núcleos.
- **Saída num núcleo só.** Somente o `loop()` (núcleo 0) escreve na USB e no WiFi. As tarefas do núcleo 1 deixam avisos em `g_event`.

## Controle

```
sensores IR ─► posição da linha (centroide, em m)
             ─► w = sinal·(Kp·pos + Ki·∫pos + Kd·dpos/dt)       (seguidor, rad/s)
                v = V_NOMINAL / (1 + SLOWDOWN·|w|)              (desacelera em curva)
             ─► roda dir. = v + w·L/2,  roda esq. = v − w·L/2  ─► rpm
             ─► PID de velocidade + Kf·rpm_ref (uma por roda) ─► Volts ─► PWM (compensado pela bateria)
```

- **Posição contínua.** A posição é o **centroide** das leituras normalizadas. Isso dá um valor contínuo entre os sensores, sem degraus, e é o que deixa a direção suave.
- **Curvas circulares.** Uma curva de raio constante pede um `w` constante. Com só P+D, sobra um erro lateral fixo na curva; o `Ki` do seguidor elimina esse erro. Na simulação, com R = 0,3 m: ~6 mm de erro sem Ki e ~0 mm com Ki.
- **Linha perdida.** O código decide pelo lugar onde a linha estava quando sumiu:
  - **Gap (falha na linha).** Se a linha sumiu perto do centro dos sensores (|pos| < `GAP_CENTER_M`), ela continua à frente. O robô segue **reto** (w = 0), com as duas rodas na mesma velocidade pelos PIDs. Ele anda no máximo `GAP_MAX_M`, medidos pelos encoders, procurando a linha. Se não a encontrar, para.
  - **Curva.** Se a linha sumiu pela lateral, ela saiu dos sensores numa curva fechada. O robô mantém a última curva por `LINE_LOST_MS` e depois para.
  - **Reencontro.** Quando a linha volta, o derivativo é reiniciado, para não dar um tranco na direção.

## Primeiros testes (nesta ordem)

1. **Sensores.** Envie `ir` com o robô parado sobre o chão e depois sobre a linha. A linha deve dar leituras altas.
2. **Calibração.** Coloque o robô sobre a linha e envie `irc`. Ele gira para os dois lados por 3 s e grava o mínimo e o máximo de cada sensor. Confira com `ir`.
3. **Sentido.** Desloque o robô com a linha à esquerda do centro e veja o sinal de `pos` no `ir`. Envie `go`. Se ele virar **para longe** da linha, envie `sign -1` e depois mude `LINE_SIGN` no `config.h`.
4. **Medida.** Meça a distância entre sensores vizinhos e corrija `IR_SPACING_M`, para a posição sair em metros reais.
5. **Ajuste.** Comece com `v 0.15`, ajuste `lkp`/`lkd` e depois aumente `v`.

## WiFi (Pico W)

Os comandos e a telemetria funcionam igual pela USB e pelo WiFi, ao mesmo tempo. A configuração fica no fim do `config.h`.

- **Modo AP** (`WIFI_AP_MODE 1`, padrão). O robô cria a rede **ACE_ROBOT**, com senha `aceminiproj`.
  1. Conecte o PC ou o celular nessa rede.
  2. Abra uma conexão TCP para **192.168.42.1, porta 23**.
- **Modo estação** (`WIFI_AP_MODE 0`). O robô entra na rede `WIFI_SSID`/`WIFI_PASS`. O IP dele aparece na USB ao ligar.

Programas para a conexão TCP:

| Sistema | Como conectar |
|---|---|
| Windows | **PuTTY**: Connection type = *Raw*, Host = 192.168.42.1, Port = 23 |
| Linux/Mac | `nc 192.168.42.1 23` |
| Android | app **Serial WiFi Terminal** ou **TCP Telnet Terminal** |

Só um cliente WiFi fica conectado por vez; uma conexão nova derruba a anterior. Envie `h` para ver a lista de comandos.

## Comandos (USB 115200 ou WiFi, terminar com Enter)

| Comando | Efeito |
|---|---|
| `go` / `st` | começa / para de seguir a linha (botão START alterna, STOP para) |
| `fwd s` | anda reto por `s` segundos na velocidade `v`, sem usar a linha (o rumo é mantido pelos encoders) |
| `walk m` | anda reto `m` metros (ex.: `walk 0.5` = 50 cm), freando para parar na marca |
| `fkh x` | `fwd`: ganho da correção de rumo |
| `fky x` | `fwd`: ganho da correção do desvio lateral |
| `acc x` | `fwd`: aceleração da rampa (m/s²) |
| `wr x` | razão raio da roda direita / esquerda (calibração) |
| `irc` | calibração dos sensores (gira no lugar por 3 s) |
| `ir` | leituras, calibração e posição da linha |
| `bat` | tensão da bateria |
| `p` | mostra todos os parâmetros |
| `h` | lista de comandos |
| `tel 0/1` | desliga/liga a telemetria CSV (`t_ms,pos_mm,w,ref1,rpm1,ref2,rpm2,u1,u2`) |
| `v x` | velocidade nominal (m/s) |
| `slow x` | redução de velocidade nas curvas |
| `sign ±1` | sentido da correção do seguidor |
| `kp ki kd kf x` | PID de velocidade das rodas (rpm → V) |
| `lkp lki lkd x` | PID do seguidor (m → rad/s) |
| `gapc x` | largura da faixa central que identifica um gap (m) |
| `gapd x` | distância máxima percorrida em linha reta num gap (m) |

Os valores mudam na hora, inclusive com o robô andando. Depois de achar bons valores, copie-os para o `config.h`.

## Andar reto com o `fwd`

O `fwd` não usa a linha. Ele calcula a odometria com os encoders e corrige dois erros:

- **Rumo:** o ângulo do robô em relação à direção inicial.
- **Desvio lateral `y`:** a distância do robô até a reta de partida. Com isso, ele volta para a reta, e não só fica paralelo a ela.

A velocidade sobe e desce em rampa, para as rodas não patinarem na partida nem na parada. Ao terminar, o robô mostra o que os encoders mediram:

```
# fwd done: encoders say 0.702 m, heading 0.31 deg, lateral 1.2 mm
```

Durante o `fwd`, a coluna `pos_mm` da telemetria mostra o desvio lateral.

### Se o robô ainda não andar reto

Compare o que o robô mostrou com o que ele fez de verdade. Marque o ponto de partida e o de chegada no chão.

1. **Os encoders dizem que foi reto (rumo ≈ 0, lateral ≈ 0), mas o robô curvou.** As rodas têm diâmetros efetivos diferentes, e o controle não tem como ver isso. Calibre com `wr`:
   - Se o robô **curva para a esquerda**, a roda direita é efetivamente maior: aumente um pouco (`wr 1.005`).
   - Se **curva para a direita**, diminua (`wr 0.995`).
   - Repita `fwd 4` até sair reto e copie o valor para `WHEEL_RIGHT_SCALE`. Variações de 0,5% já fazem diferença em 1 m.
2. **Os encoders mostram rumo ou desvio lateral grandes.** O controle não está corrigindo bem. Aumente `fkh` (rumo) e `fky` (desvio lateral). Se o robô zigue-zaguear, diminua.
3. **O robô desvia logo na partida.** Uma roda arrancou antes da outra, ou patinou. Diminua `acc`, por exemplo `acc 0.3`.
4. **O robô desvia de forma diferente a cada vez.** A causa é mecânica: roda folgada, roda-boba travando ou piso irregular. Ajuste o mecanismo antes de mexer nos ganhos.
