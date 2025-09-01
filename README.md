# micro-ros
Embora plataformas de computação de alto desempenho lidem com tarefas complexas de percepção e planejamento, é o hardware de baixo nível — microcontroladores, sensores e atuadores — que interage fisicamente com o mundo real.

Apesar de sua importância, a integração desses dispositivos embarcados com sistemas robóticos de nível superior tem sido tradicionalmente um processo fragmentado e difícil.

Diferentes linguagens de programação. Diferentes ferramentas de desenvolvimento. Diferentes protocolos de comunicação.

É aqui que o **micro-ROS** entra em ação.

O Micro-ROS preenche a lacuna entre o software de robótica moderno (ROS 2) e o desenvolvimento de sistemas embarcados.

Ele permite que você escreva nós ROS 2 que rodam em microcontroladores, dando ao seu código embarcado acesso nativo a objetos ROS: publicadores, assinantes, serviços e muito mais.

Em vez de ter sistemas separados para "código robótico" e "firmware", o micro-ROS unifica o desenvolvimento em toda a pilha robótica — desde o planejamento de alto nível até as leituras de sensores de baixo nível.

Isso significa:

* Você pode enviar mensagens ROS 2 diretamente de sensores embarcados.
* Você pode controlar atuadores a partir de um gráfico ROS centralizado.
* Você pode estruturar seu robô como um sistema coeso e distribuído, onde até mesmo microcontroladores atuam como nós ROS 2 completos.
* Isso muda fundamentalmente a forma como os robôs são construídos.
* O Micro-ROS permite que você trate dispositivos embarcados como cidadãos de primeira classe dentro de uma arquitetura robótica — não mais caixas-pretas isoladas, mas participantes em tempo real de um ecossistema compartilhado.

## A necessidade do microROS
Em geral, podemos concordar que a maioria dos sistemas robóticos consiste em pelo menos duas camadas:

* Uma camada de computação de alto nível que executa tarefas complexas como percepção, tomada de decisão e coordenação
* Uma camada de controle de baixo nível que gerencia o hardware, como sensores, atuadores e loops em tempo real.

A camada de alto nível normalmente é executada em um PC ou SBC (por exemplo, Raspberry Pi, Jetson Nano), enquanto a camada de baixo nível geralmente depende de microcontroladores (por exemplo, STM32, ESP32). Esses microcontroladores são essenciais para tarefas como:

* Controle de motores
* Aquisição de dados de sensores
* Manipulação de PWM e ADC
* Loops de feedback em tempo real

No entanto, a integração desses componentes embarcados com sistemas robóticos de alto nível tem sido tradicionalmente um enorme desafio. As abordagens tradicionais envolvem a criação de protocolos personalizados, pontes seriais ou wrappers de comunicação mínimos. Esses métodos são difíceis de manter, carecem de padronização e, frequentemente, estão sujeitos a erros.

É aqui que o microROS entra em ação: ele preenche a lacuna entre o ROS 2 e os sistemas embarcados, unificando o desenvolvimento em toda a pilha robótica.

O MicroROS é um projeto de código aberto que estende os recursos do ROS 2 para execução em **dispositivos com recursos extremamente limitados**. Ele permite que microcontroladores funcionem como participantes de primeira classe no ecossistema ROS 2.

**Por que não simplesmente executar o ROS2 em microcontroladores?**

O ROS 2 pressupõe vários recursos nem sempre disponíveis em microcontroladores:

* Um sistema operacional com suporte a multithreading
* Alocação dinâmica de memória
* Rede baseada em Ethernet ou IP
* Acesso ao sistema de arquivos

Os microcontroladores geralmente operam com dezenas ou centenas de quilobytes de RAM, com velocidades de CPU na casa das dezenas de MHz. A maioria dos clientes ROS 2 consome muitos recursos para esses ambientes.

O MicroROS aborda esse desafio adaptando a arquitetura e o modelo de comunicação do ROS 2 para funcionar com eficiência em hardware tão limitado. O MicroROS reutiliza o máximo possível da arquitetura em camadas do ROS 2, dentro dos limites dos sistemas embarcados.

A imagem abaixo mostra a arquitetura do framework:

![U1_U2_micro-ROS_architecture.png](https://github.com/marcospontoexe/micro-ros/blob/main/imagens/U1_U2_micro-ROS_architecture.png).

Sabemos que há muitas caixas, então vamos ver se conseguimos decompor o diagrama.

No cerne do microROS está uma arquitetura de dois componentes:

* um cliente leve rodando no microcontrolador e
* um agente mais poderoso rodando em um sistema host.

![U2_micro-ROS_architecture -dual_component](https://github.com/marcospontoexe/micro-ros/blob/main/imagens/U2_micro-ROS_architecture%20-dual_component.png)

O microcontrolador **não utiliza o mesmo protocolo ROS 2 DDS** (Data Distribution Service) usado em um sistema completo. Em vez disso, ele utiliza uma variante chamada **DDS-XRCE**, que significa DDS (Extremely Resource-Constrained Environments). Este protocolo minimiza a sobrecarga e permite que o microcontrolador se comunique com o ecossistema ROS 2 usando menos memória e poder de processamento.

![U2_micro-ROS_architecture - protocols](https://github.com/marcospontoexe/micro-ros/blob/main/imagens/U2_micro-ROS_architecture%20-%20protocols.png)

Basicamente, a pilha microROS é dividida em duas partes:

* Lado do cliente (no MCU): Lida com o código do usuário, API de comunicação e execução em tempo real
* Lado do agente (no host): Traduz mensagens DDS-XRCE para tópicos e serviços ROS 2 DDS

O cliente microROS é construído sobre um RTOS — normalmente FreeRTOS, Zephyr ou NuttX — que fornece agendamento de tarefas, recursos em tempo real e serviços básicos do sistema, como temporizadores e threads.

![U2_micro-ROS_architecture - RTOS](https://github.com/marcospontoexe/micro-ros/blob/main/imagens/U2_micro-ROS_architecture%20-%20RTOS.png)

Acima do RTOS está o **Cliente Micro XRCE-DDS**, um componente responsável por gerenciar a conexão com o **Agente ROS 2** externo. Ele lida com a serialização de mensagens e a manutenção de sessões, atuando como um tradutor entre o microcontrolador e a rede ROS 2 de nível superior.

Para facilitar o desenvolvimento e trazer familiaridade aos desenvolvedores ROS, o microROS fornece uma API baseada em C semelhante à biblioteca cliente C padrão do ROS 2 (**rcl**).

Além disso, um conjunto de funções utilitárias e abstrações é oferecido por meio do rclc, o que simplifica tarefas como criação de nós, configuração de executores e gerenciamento de tópicos. Isso significa que os desenvolvedores podem escrever código para o microcontrolador de uma forma estilística e estruturalmente consistente com a forma como escreveriam aplicações ROS 2 padrão.

![U2_micro-ROS_architecture - language](https://github.com/marcospontoexe/micro-ros/blob/main/imagens/U2_micro-ROS_architecture%20-%20language.png)

O componente **agente** do microROS normalmente é executado em um computador desktop ou de placa única com uma instalação completa do ROS 2.

Este agente monitora conexões de microcontroladores por meio de interfaces como **UART, USB ou UDP via Wi-Fi**.

![U2_micro-ROS_architecture - agent](https://github.com/marcospontoexe/micro-ros/blob/main/imagens/U2_micro-ROS_architecture%20-%20agent.png)

Uma vez estabelecida a conexão, o **agente se torna o proxy para o microcontrolador** no grafo ROS 2.

Da perspectiva de outros nós ROS 2, não há diferença entre um nó rodando em um PC e um rodando em um microcontrolador; o agente microROS garante uma integração perfeita.
Essa arquitetura permite que os desenvolvedores construam sistemas robóticos distribuídos onde o comportamento de alto nível é executado em processadores potentes, enquanto reações rápidas em tempo real e o controle direto do hardware são gerenciados por microcontroladores. A separação de responsabilidades entre o microcontrolador e o host também melhora a modularidade e a confiabilidade do sistema.

Em última análise, o microROS capacita os roboticistas a implementar sistemas totalmente integrados que abrangem tanto o planejamento de alto nível quanto a execução de baixo nível.

Ele agiliza o desenvolvimento, promove a reutilização de código e mantém os princípios básicos do ROS. Mesmo nos menores componentes de um robô.

Ao trabalhar com o microROS, o fluxo de trabalho geral se parece com o seguinte:

* Escrever o código da aplicação usando a API rclc
* Configurar o middleware (parâmetros DDS-XRCE, transporte, QoS)
* Instalar o firmware no microcontrolador
* Executar o Agente microROS em um host Linux
* Monitorar e interagir usando ferramentas e nós do ROS 2

## Exemplo prático
Veja um sistema de comunicação entre dois nós ROS: um rodando em um **microcontrolador ESP32** e o outro rodando no seu **computador**.

Simularemos uma interação de pingue-pongue:

* O nó ESP32 (**cliente microROS**) envia uma mensagem "**ping**" para um tópico.
* Um **nó ROS2** recebe o **ping** e responde com um "**pong**".
* O **ESP32** recebe o **pong** e envia outro **ping** — repetindo o ciclo.

Para tornar isso possível, você configurará três componentes principais:

1. O **cliente micro-ROS do ESP32** — Um pequeno programa no ESP32 que se conecta ao Wi-Fi, publica mensagens em um tópico **/ping** e escuta as respostas no tópico **/pong**.
2. O **Agente micro-ROS** — Uma ferramenta especial que atua como uma ponte entre o ESP32 e a rede ROS 2 do seu computador. Ele escuta as mensagens **UDP do ESP32** e as encaminha para o ecossistema ROS 2.
3. O [**nó "Pong" do ROS 2**](https://github.com/marcospontoexe/micro-ros/blob/main/exemplos/pong_reply_node/pong_reply_node/pong.py) — Um nó ROS2 típico em execução no seu **computador** que escuta **pings**, registra as mensagens e envia respostas do tipo **pong**.

O diagrama abaixo mostra o fluxo de comunicação:

![U2_ping_pong_workflow](https://github.com/marcospontoexe/micro-ros/blob/main/imagens/U2_ping_pong_workflow.png)

## Agente Micro Ros
O Agente micro-ROS é construído usando um conjunto de pacotes compatíveis com ROS 2 que gerenciam redes de baixo nível, tradução DDS, serialização e protocolos de comunicação.

Você usará os scripts micro_ros_setup para automatizar o processo de preparação, construção e instalação do agente.

Agora criamos um novo espaço de trabalho que hospedará as ferramentas de configuração do micro-ROS (**Esse será um novo espaço de trabalho, antão não deve ser criado dentro do diretório ros2_ws, e sim um diretório a cima, por exemplo /home/user/microros_ws/**):

```shell
cd ~
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```

A linha acima baixa os scripts de configuração, correspondentes à sua distribuição ROS 2 (por exemplo, humble), para a pasta src do seu novo espaço de trabalho.

Antes de compilar qualquer coisa, precisamos garantir que todas as dependências necessárias estejam disponíveis. Isso inclui bibliotecas usadas para comunicação, compilação e suporte a transporte:

```shell
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
```

Agora, crie o espaço de trabalho de configuração do micro-ROS e crie-o para que você possa usar os comandos de configuração incluídos:

```shell
cd ~/microros_ws/
colcon build
source install/local_setup.bash
```

Neste ponto, você já tem as ferramentas de configuração disponíveis. Agora, criaremos o espaço de trabalho do agente usando um script fornecido pelo micro-ROS: 

```shell
cd ~/microros_ws/
ros2 run micro_ros_setup create_agent_ws.sh
```

Este comando prepara um espaço de trabalho aninhado dentro do seu atual.

Ele baixa e configura todo o código-fonte necessário para o agente em si — incluindo bibliotecas de transporte, serialização, pontes DDS e o executável micro_ros_agent.

Vamos construir o agente agora:

```shell
cd ~/microros_ws/
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

Quando isso terminar, seu sistema terá o agente instalado e pronto para ser executado.

###  Executando o agente microROS
Agora, vamos iniciar o Agente, que escuta mensagens **UDP do ESP32** e as insere no ecossistema ROS 2.

```shell
cd ~/microros_ws
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Isso iniciará o agente usando UDP sobre IPv4, escutando na porta 8888, exatamente como o seu ESP32 estará configurado para usar.

No terminal, você deverá ver uma saída como a seguinte:

```shell
[1679041670.087954] info     | UDPv4AgentLinux.cpp | init                     | running...             | port: 8888
[1679041670.088013] info     | Root.cpp            | set_verbose_level        | logger setup           | verbose_level: 4
```

Isso significa que o agente foi iniciado com sucesso e está escutando na porta 8888.

Você deve prestar atenção à direção desta porta, pois é importante estabelecer uma comunicação WiFi.

### O agente IP
Para estabelecer um link de comunicação bem-sucedido entre o seu ESP32 e o agente micro-ROS em execução no seu computador, precisamos de duas informações essenciais:

* O **endereço IP do computador (host)** onde o agente está em execução.
* O **número da porta** onde o agente está escutando as mensagens micro-ROS recebidas.

Pense no seu endereço IP como o "endereço" do seu computador na sua rede local. Mas, assim como um prédio de apartamentos tem muitas portas e caixas de correio diferentes, seu computador tem muitos serviços em execução em portas diferentes. Um número de porta é como uma caixa de correio ou número de quarto específico — ele ajuda a encaminhar a mensagem para o aplicativo correto no seu computador.

Quando você conecta seu ESP32 ao Wi-Fi, ele se torna um dispositivo na sua rede local (LAN), assim como seu laptop ou desktop. Cada dispositivo conectado ao mesmo roteador recebe um endereço IP local exclusivo — normalmente algo como 192.168.0.24. Esses endereços permitem que os dispositivos "vejam" e se comuniquem entre si dentro da rede.

![U2_IP_adress](https://github.com/marcospontoexe/micro-ros/blob/main/imagens/U2_IP_adress.png)

O UDP (User Datagram Protocol) é um dos principais protocolos do conjunto de protocolos de internet. Ao contrário do TCP, ele não possui conexão, o que significa que envia mensagens (chamadas datagramas) sem estabelecer uma conexão confiável. Ele não garante a entrega ou a ordem, mas é extremamente rápido e leve — ideal para tarefas de robótica em tempo real, como atualizações de sensores, comandos de atuadores ou mensagens simples de pingue-pongue, que são pequenas ou atualizadas com frequência e não exigem confirmação.

É por isso que o micro-ROS frequentemente usa o UDP: ele permite a transmissão de dados de baixa latência entre o agente (no seu PC) e o microcontrolador (ESP32), especialmente em ambientes embarcados com restrições.

Na seção anterior, definimos manualmente a porta como 8888. Este número não foi escolhido arbitrariamente: embora tecnicamente qualquer porta UDP livre (portas são números de 0 a 65535) possa ser usada, 8888 se tornou uma espécie de convenção informal na comunidade de robótica ao configurar comunicações micro-ROS. Isso mantém a previsibilidade e evita conflitos com outros serviços.

Resta determinar o endereço IP do seu computador na rede Wi-Fi local. Este é o endereço que o cliente ESP32 usará para enviar mensagens UDP ao agente.

## O Cliente micro-ROS 

![microclient.png](https://github.com/marcospontoexe/micro-ros/blob/main/imagens/U2_ping_pong_workflow%20-%20microclient.png)

No cerne de qualquer sistema robótico baseado em ROS 2 está a ideia de comunicação distribuída: sensores, atuadores e nós lógicos, todos trocando dados por meio de tópicos, serviços e ações.

Em uma máquina desktop, isso normalmente significa iniciar um nó ROS 2 — um processo que interage com outros processos por meio do protocolo DDS (Serviço de Distribuição de Dados) subjacente. Esses nós são executados no espaço do usuário, sobre um sistema operacional completo, e dependem de muitas camadas de software, de bibliotecas de middleware a threads POSIX.

Mas robôs reais não são construídos apenas com PCs. Eles exigem microcontroladores — chips pequenos, energeticamente eficientes e baratos que se conectam diretamente a motores, sensores, botões e displays. Esses microcontroladores, no entanto, não podem executar o ROS 2. Eles não possuem um sistema operacional, não podem arcar com a sobrecarga de memória do DDS e não podem participar do gráfico do ROS 2 da mesma forma que um nó Linux completo.

Para resolver isso, o micro-ROS foi criado. Trata-se de uma adaptação enxuta e embarcada da pilha ROS 2 que permite que microcontroladores — como o seu ESP32 — se comportem como nós ROS 2. Eles não executam o DDS, mas podem participar do gráfico ROS 2 por meio do Agente micro-ROS, que atua como uma ponte.

O cliente micro-ROS (o código executado no ESP32) se conecta ao Agente usando um protocolo de comunicação leve (geralmente via serial ou UDP), e o Agente encaminha as mensagens para a rede DDS em nome do microcontrolador.

Portanto, quando falamos do cliente micro-ROS, estamos nos referindo ao conjunto de softwares executado diretamente no ESP32 que:

* Cria um nó ROS
* Publica e assina tópicos do ROS 2
* Comunica-se com o Agente usando XRCE-DDS
* Comporta-se como um nó ROS 2 completo da perspectiva de outros nós no sistema

Este código do cliente é escrito em C e inclui todas as camadas essenciais do ROS 2: rcl, rclc, rmw, definições de mensagens e manipuladores de transporte — mas de forma reduzida e otimizada para executar em algumas centenas de quilobytes de memória.

### Micro-ROS Arduino
Para executar o código do cliente micro-ROS no ESP32, precisamos compilar esse código em uma imagem de firmware binária e, em seguida, instalá-la na placa.

É aqui que o ecossistema Arduino entra em cena.

O ESP32 é compatível com o sistema de compilação do Arduino — uma coleção de scripts, bibliotecas e cadeias de ferramentas que tornam o desenvolvimento embarcado mais fácil e portátil.

A CLI do Arduino é uma interface de linha de comando que fornece acesso a toda a cadeia de ferramentas do Arduino sem a necessidade de usar uma interface gráfica. Essa ferramenta nos permite:

* Compilar código compatível com o Arduino usando a cadeia de ferramentas do ESP32
* Incluir automaticamente as bibliotecas e configurações corretas da placa
* Carregar o binário resultante para o ESP32 via USB
* Integrar tudo isso em scripts ou fluxos de trabalho de shell

Mas, mais importante, no contexto do micro-ROS, a CLI do Arduino nos fornece uma maneira estruturada e repetível de compilar e instalar firmware compatível com ROS em um microcontrolador. Sem ela, precisaríamos configurar o gcc manualmente, manipular scripts de linker e instalar binários manualmente.

A CLI cuida de tudo isso para nós — ela abstrai os detalhes de configuração da placa e fornece uma interface limpa para construir e implementar firmware.

Para configurar corretamente a interface Arduino CLI na plataforma The Construct, vamos primeiro criar um arquivo de configuração que usaremos para configurar o ambiente neste curso.

```shell
cd ~
touch arduino_prepare.sh
chmod +x arduino_prepare.sh
```

Agora, vamos adicionar o seguinte código ao arquivo que você acabou de criar.

```sh
#!/bin/bash
CONFIG=/tmp/cli.yaml

arduino-cli config init --config-file $CONFIG
arduino-cli config set directories.user /opt/arduino --config-file $CONFIG
arduino-cli config set directories.data /opt/arduino --config-file $CONFIG
arduino-cli config set directories.downloads /opt/arduino/staging --config-file $CONFIG
arduino-cli config add board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json --config-file $CONFIG
arduino-cli core update-index --config-file $CONFIG
arduino-cli core install esp32:esp32 --config-file $CONFIG

echo "✅ Arduino CLI is ready."
```

Este é um script de configuração que configura o [Arduino CLI](https://arduino.github.io/arduino-cli/1.3/) para uso com a plataforma da placa ESP32.

```sh
CONFIG=/tmp/cli.yaml
```

Isso define uma variável **CONFIG** apontando para o arquivo temporário `**/tmp/cli.yaml**`, que será usado para armazenar a configuração do Arduino CLI.

```sh
arduino-cli config init --config-file $CONFIG
```

Isso inicializa um novo arquivo de configuração do Arduino CLI no local especificado por `**$CONFIG**`.

```sh
arduino-cli config set directories.user /opt/arduino --config-file $CONFIG
arduino-cli config set directories.data /opt/arduino --config-file $CONFIG
arduino-cli config set directories.downloads /opt/arduino/staging --config-file $CONFIG
```

Estas linhas configuram diferentes caminhos de diretório:

* **directories.user**: onde dados do usuário, como esboços e bibliotecas, são armazenados.
* **directories.data**: dados internos do Arduino (como núcleos instalados).
* **directories.downloads**: arquivos temporários de download durante as instalações.

Todos eles são definidos como subdiretórios em **/opt/arduino**.

```sh
arduino-cli config add board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json --config-file $CONFIG
```

Isso adiciona uma URL adicional ao índice do Gerenciador de Placas, especificamente para placas ESP32 fornecidas pela Espressif.

Isso é necessário porque as placas ESP32 não são incluídas por padrão.

```sh
arduino-cli core update-index --config-file $CONFIG
```

Isso atualiza a lista de pacotes de placas disponíveis (núcleos) usando os URLs configurados.

```sh
arduino-cli core install esp32:esp32 --config-file $CONFIG
```

Isso instala o núcleo ESP32 para a plataforma Arduino, tornando possível compilar e carregar código para placas ESP32.

Agora que temos uma maneira de compilar e enviar código, a questão é: qual código estamos compilando?

O cliente micro-ROS não é apenas um esboço comum do Arduino. Ele depende de todo um ecossistema de bibliotecas do lado do cliente ROS 2.

Essas bibliotecas incluem as definições de mensagens (std_msgs, sensor_msgs), a API principal do ROS (rcl), o executor leve (rclc) e a ponte de middleware (rmw_microxrcedds), bem como a lógica de transporte e serialização (como microxrcedds_client e micro_cdr).

Reunir todas essas peças manualmente seria uma tarefa árdua. Elas devem ser compiladas na ordem correta, vinculadas corretamente e interoperar dentro do ambiente restrito de um microcontrolador.

É aqui que entra a biblioteca [**micro_ros_arduino**](https://github.com/micro-ROS/micro_ros_arduino).

A biblioteca micro_ros_arduino é um pacote cuidadosamente elaborado que reúne todos os componentes micro-ROS necessários em um formato compatível com o sistema de construção do Arduino. Ela fornece wrappers e APIs compatíveis com o Arduino que permitem escrever código no estilo familiar dos sketches do Arduino, enquanto ainda acessa o poder do ROS 2. No entanto, é importante entender que:

**Esta biblioteca não é um aprimoramento de uso geral do Arduino.**

Ela não fornece recursos como **digitalWrite()** ou **analogRead()**. Esses recursos ainda vêm do núcleo do Arduino ESP32, que é um pacote de plataforma separado mantido pela Espressif. O que o micro_ros_arduino faz é integrar a funcionalidade do ROS 2 ao modelo de programação do Arduino, permitindo que você construa nós ROS 2 em hardware que nunca foi projetado para essa tarefa.

Nesse sentido, o micro_ros_arduino é a ligação entre o mundo embarcado e o mundo ROS — e permite que você construa uma imagem de firmware compatível com o Arduino e com suporte ao ROS 2.

**Para usar o micro_ros_arduino, você precisa cloná-lo na pasta de bibliotecas do seu Arduino.**

Isso normalmente é feito manualmente, pois a biblioteca depende de uma ramificação específica vinculada à sua distribuição ROS 2 (por exemplo, humble).

No terminal, execute:

```shell
mkdir -p ~/microROS_Arduino/libraries
cd ~/microROS_Arduino/libraries
git clone -b humble https://github.com/micro-ROS/micro_ros_arduino.git
```

Isso coloca a integração micro-ROS do Arduino em uma estrutura que a CLI do Arduino entende.

Agora, ao compilar seu esboço, você pode especificar esta pasta usando a flag --libraries, e a CLI do Arduino a incluirá no processo de construção.

### Criando um cliente micro ros
Com a biblioteca instalada, você pode criar o firmware — o código do cliente micro-ROS.

Com a CLI do Arduino, esta etapa consiste na criação de um esboço do Arduino — um arquivo .ino — que você escreve em C++.

No terminal, execute os seguintes comandos para criar o arquivo .ino.

```shell
mkdir -p ~/microROS_Arduino/microros_ping_node
cd ~/microROS_Arduino/microros_ping_node
touch microros_ping_node.ino
chmod +x microros_ping_node.ino
```

Isso deve ter criado o arquivo onde estamos colocando a lógica do cliente microROS.

Este cliente micro-ROS deve ser executado na sua placa ESP32 e implementar um padrão de comunicação pingue-pongue simples usando mensagens ROS 2.

O ESP32 deve enviar uma mensagem no tópico **/ping** para um nó ROS 2 em execução no seu computador host. Esse nó responde no tópico **/pong**. Quando o ESP32 receber a resposta, ele deve enviar uma nova mensagem "ping". Essa troca continua indefinidamente.

Agora, abra o arquivo que você acabou de criar e inclua o seguinte código:

```c
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

char *ssid = "YourSSID";
char *password = "YourPassword";
char *agent_ip = "YourAgentIP"; 
uint32_t agent_port = 8888; // Default micro-ROS agent port

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_publisher_t ping_pub;
rcl_subscription_t pong_sub;
rclc_executor_t executor;

std_msgs__msg__String ping_msg;
std_msgs__msg__String pong_msg;

char ping_data[64];
char data_buf[128];
int ping_count = 0;

#if defined(LED_BUILTIN)
#define LED_PIN LED_BUILTIN
#else
#define LED_PIN 2
#endif

// --- Helpers ----------------------------------------------------

bool connectWiFi(unsigned long timeout_ms = 15000) {
  Serial.print("[WIFI] Connecting to SSID: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > timeout_ms) {
      Serial.println("\n[WIFI] Connect timeout");
      return false;
    }
    Serial.print("...");
    delay(500);
  }
  Serial.println();
  Serial.print("[WIFI] Connected. IP Address: ");
  Serial.println(WiFi.localIP());
  return true;
}

void pingAgent(unsigned long ping_interval_ms = 1000) {
  Serial.print("[MICROROS] Pinging agent at ");
  Serial.print(agent_ip);
  Serial.print(":");
  Serial.print(agent_port);
  Serial.print(" ");
  // try every ping_interval_ms until success
  while (rmw_uros_ping_agent(ping_interval_ms, 1) != RMW_RET_OK) {
    Serial.print("...");
    delay(200);
  }
  Serial.println("\n[MICROROS] Agent found!");
}

// --- Callbacks --------------------------------------------------

// Callback when receiving a /pong
void pong_callback(const void *msgin) {
  Serial.println("[ESP32] pong_callback triggered");
  const std_msgs__msg__String *pong = (const std_msgs__msg__String *)msgin;
  Serial.printf("[ESP32] Got PONG: '%s'\n", pong->data.data);

  // Send next ping
  ping_count++;
  snprintf(ping_data, sizeof(ping_data), "ping #%d", ping_count);
  ping_msg.data.data = ping_data;
  ping_msg.data.size = strlen(ping_data);
  ping_msg.data.capacity = sizeof(ping_data);

  Serial.printf("[ESP32] Sending PING: '%s'\n", ping_data);
  rcl_publish(&ping_pub, &ping_msg, NULL);
  delay(1000);
}

// --- Setup & Loop ----------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  // LED blink to signal boot
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);

  // Wi-Fi
  if (!connectWiFi()) {
    Serial.println("[SETUP] WiFi failed, halting.");
    while (true) {
      delay(1000);
    }
  }

  // micro-ROS transport & agent
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  pingAgent();
  Serial.println(" Agent is up!");

  // rclc Initialization
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_ping_node", "", &support);
  Serial.println("Node initialized");

  // Publisher & Subscription
  rclc_publisher_init_default(
      &ping_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/ping");
  Serial.println("Success publisher");

  std_msgs__msg__String__init(&pong_msg);
  pong_msg.data.data =
      data_buf; // you need a char buffer declared like: char data_buf[128];
  pong_msg.data.capacity = sizeof(data_buf);
  pong_msg.data.size = 0;

  rclc_subscription_init_default(
      &pong_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/pong");
  Serial.println("Success subscriber");

  // Prepare incoming message object
  std_msgs__msg__String__init(&ping_msg);
  ping_msg.data.data = ping_data;
  ping_msg.data.capacity = sizeof(ping_data);
  ping_msg.data.size = 0;

  // Executor & Timer
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &pong_sub, &pong_msg,
                                 &pong_callback, ON_NEW_DATA);
  Serial.println("Success excecutor");

  // Send first ping
  ping_count = 1;
  snprintf(ping_data, sizeof(ping_data), "ping #%d", ping_count);
  ping_msg.data.data = ping_data; // <-- this line is critical
  ping_msg.data.size = strlen(ping_data);
  ping_msg.data.capacity = sizeof(ping_data);
  Serial.printf("[ESP32] Sending initial PING: '%s'\n", ping_data);
  rcl_publish(&ping_pub, &ping_msg, NULL);

  Serial.println("First ping sent");
}

void loop() {
  if (ping_count == 1) {
    Serial.println("We exited the setup");
  }
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
```

Vamos revisar cuidadosamente esse código juntos.

```c
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>
```

Estas diretivas #include trazem as bibliotecas necessárias para:

* WiFi.h: para conectar o ESP32 a uma rede sem fio.
* micro_ros_arduino.h: para configurar a comunicação micro-ROS via UDP.
* rcl e rclc: as bibliotecas de cliente ROS da camada C para inicializar nós, publicadores e assinaturas.
* std_msgs/msg/string.h: para usar o tipo de mensagem String padrão do ROS 2.

```c
char *ssid = "YourSSID";
char *password = "YourPassword";
char *agent_ip = "YourAgentIP"; 
uint32_t agent_port = 8888; // Default micro-ROS agent port
```

Estas são variáveis ​​de configuração globais para Wi-Fi e o endereço IP e a porta do Agente micro-ROS.

O agent_ip deve corresponder à máquina host que executa o micro_ros_agent.

```c
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_publisher_t ping_pub;
rcl_subscription_t pong_sub;
rclc_executor_t executor;
```

Estas são as principais estruturas de dados no sistema micro-ROS:

* rcl_node_t: representa o nó no grafo ROS 2 (esp32_ping_node).
* rclc_support_t: contém o contexto de inicialização e as configurações do alocador de memória.
* rcl_allocator_t: define como a memória é gerenciada para mensagens.
* rcl_publisher_t / rcl_subscription_t: as interfaces para os tópicos /ping e /pong.
* rclc_executor_t: gerencia o agendamento de retornos de chamada quando novos dados são recebidos.

```c
std_msgs__msg__String ping_msg;
std_msgs__msg__String pong_msg;

char ping_data[64];
char data_buf[128];
int ping_count = 0;
```

Aqui, definimos buffers e contadores de mensagens.

* ping_msg e pong_msg são estruturas em C para o tipo de mensagem ROS std_msgs/String.
* ping_data e data_buf são matrizes de caracteres em C (buffers) para armazenar dados de string para mensagens de entrada e saída.
* ping_count é um contador simples para rotular cada mensagem de ping (por exemplo, "ping #1", "ping #2", etc.).

```c
#if defined(LED_BUILTIN)
#define LED_PIN LED_BUILTIN
#else
#define LED_PIN 2
#endif
```

Esta é a configuração dos pinos do LED.

Preste atenção especial à configuração do LED!
Você precisará disso mais adiante nesta unidade.

Esta seção define qual pino GPIO o LED onboard usa.

Se LED_BUILTIN for definido pela plataforma, use-o; caso contrário, use o padrão para GPIO 2 (que é o GPIO atribuído ao LED no NodeMCU).

```c
void pingAgent(unsigned long ping_interval_ms = 1000) {
  Serial.print("[MICROROS] Pinging agent at ");
  Serial.print(agent_ip);
  Serial.print(":");
  Serial.print(agent_port);
  Serial.print(" ");
  while (rmw_uros_ping_agent(ping_interval_ms, 1) != RMW_RET_OK) {
    Serial.print("...");
    delay(200);
  }
  Serial.println("\n[MICROROS] Agent found!");
}
```

Esta função utiliza rmw_uros_ping_agent() para verificar ativamente se o Agente está online e acessível pela rede.

Ela imprime pontos de status até que uma conexão seja confirmada.

```c
void pong_callback(const void *msgin) {
  Serial.println("[ESP32] pong_callback triggered");
  const std_msgs__msg__String *pong = (const std_msgs__msg__String *)msgin;
  Serial.printf("[ESP32] Got PONG: '%s'\n", pong->data.data);

  // Send next ping
  ping_count++;
  snprintf(ping_data, sizeof(ping_data), "ping #%d", ping_count);
  ping_msg.data.data = ping_data;
  ping_msg.data.size = strlen(ping_data);
  ping_msg.data.capacity = sizeof(ping_data);

  Serial.printf("[ESP32] Sending PING: '%s'\n", ping_data);
  rcl_publish(&ping_pub, &ping_msg, NULL);
  delay(1000);
}
```

Esta função é chamada quando uma mensagem chega ao tópico /pong. Ela:

* Imprime a mensagem recebida.
* Incrementa o contador.
* Formata uma nova mensagem de ping.
* Publica-a no tópico `/ping usando rcl_publish()`.

Observe a conversão de ponteiro de `const void *` para `std_msgs__msg__String *`. Isso é necessário porque o retorno de chamada usa uma interface de ponteiro genérica em C.

```c
allocator = rcl_get_default_allocator();
rclc_support_init(&support, 0, NULL, &allocator);
rclc_node_init_default(&node, "esp32_ping_node", "", &support);
```

Inicializa estruturas internas do micro-ROS, usando alocadores de memória padrão, e cria o nó esp32_ping_node.

```c
rclc_publisher_init_default(&ping_pub, &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/ping");

std_msgs__msg__String__init(&pong_msg);
pong_msg.data.data = data_buf;
pong_msg.data.capacity = sizeof(data_buf);
pong_msg.data.size = 0;

rclc_subscription_init_default(&pong_sub, &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/pong");
```

Estas linhas configuram:

* O publicador /ping.
* O assinante /pong.
* Buffers de memória para a mensagem pong_msg recebida.

```c
rclc_executor_init(&executor, &support.context, 1, &allocator);
rclc_executor_add_subscription(&executor, &pong_sub, &pong_msg, &pong_callback, ON_NEW_DATA);
```

O executor é um pequeno escalonador que verifica se há novas mensagens recebidas e chama o retorno de chamada relevante quando uma chega.

Aqui, ele está configurado para executar pong_callback quando uma nova mensagem /pong chega.

```c
ping_count = 1;
snprintf(ping_data, sizeof(ping_data), "ping #%d", ping_count);
ping_msg.data.data = ping_data;
ping_msg.data.size = strlen(ping_data);
ping_msg.data.capacity = sizeof(ping_data);
Serial.printf("[ESP32] Sending initial PING: '%s'\n", ping_data);
rcl_publish(&ping_pub, &ping_msg, NULL);
```

Este bloco envia "manualmente" o primeiro ping para iniciar o ciclo.

Os pings subsequentes são tratados automaticamente no retorno de chamada.

```c
void loop() {
  if (ping_count == 1) {
    Serial.println("We exited the setup");
  }
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
```

Este é o loop principal de tempo de execução.

A chamada para rclc_executor_spin_some() executa o executor e permite que ele verifique se há novas mensagens. A condição com ping_count é apenas uma impressão temporária para confirmar que a configuração foi concluída. É apenas uma etapa de depuração.

### Building the Client with Arduino CLI
Depois que seu esboço estiver escrito e salvo, você o compilará usando a CLI do Arduino.

Esse processo envolve invocar a CLI com a configuração correta da placa, especificar o diretório do seu esboço e vinculá-lo à biblioteca micro-ROS do Arduino.

No terminal, execute os seguintes comandos:

```shell
~/arduino_prepare.sh
cd ~/microROS_Arduino/microros_ping_node
arduino-cli compile \
  --config-file /tmp/cli.yaml \
  --fqbn esp32:esp32:nodemcu-32s \
  --libraries ~/microROS_Arduino/libraries \
  --output-dir ~/microROS_Arduino/microros_ping_node/build \
  ~/microROS_Arduino/microros_ping_node
```

Vamos revisar as diferentes partes deste comando:

* `~/arduino_prepare.sh`. Primeiro, executamos o arquivo de configuração necessário para preparar o ambiente.
* `cd ~/microROS_Arduino/microros_ping_node`. Em seguida, movemos para o diretório onde o arquivo .ino está armazenado.
* Finalmente, usamos o comando arduino-cli compile para compilar o firmware.
* Aqui, a flag `--fqbn` informa à CLI para qual placa você está compilando — neste caso, a NodeMCU ESP32.
* A flag `--libraries` inclui a biblioteca micro-ROS Arduino que você instalou anteriormente.
* A flag `--output-dir` especifica onde colocar os artefatos de compilação.

Ao executá-la, a CLI do Arduino invoca a cadeia de ferramentas do ESP32 para compilar seu esboço e todas as suas dependências.

Isso inclui o tempo de execução do micro-ROS, o suporte a tipos de mensagens e quaisquer componentes de middleware.



```sh
#!/bin/bash
CONFIG=/tmp/cli.yaml

arduino-cli config init --config-file /tmp/cli.yaml
arduino-cli config set directories.user /opt/arduino --config-file $CONFIG
arduino-cli config set directories.data /opt/arduino --config-file $CONFIG
arduino-cli config set directories.downloads /opt/arduino/staging --config-file $CONFIG
arduino-cli config add board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json --config-file $CONFIG
arduino-cli core update-index --config-file $CONFIG
arduino-cli core install esp32:esp32 --config-file $CONFIG

echo "✅ Arduino CLI is ready."
```