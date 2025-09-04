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


### Instalando o firmware no ESP32
A etapa final é a flash — copiar os binários compilados para a memória flash do ESP32. É isso que torna seu código "vivo" no chip.

Então, para esta seção, vamos pegar o hardware correto, no nosso caso a **ESP32-S NodeMCU Wroom**.

Ao conectar uma placa ESP32 ao seu computador usando um cabo USB, você não está se conectando diretamente ao chip ESP32. Em vez disso, você está se comunicando por meio de um chip conversor USB-serial soldado à placa. Este chip converte sinais USB em dados seriais que o ESP32 consegue entender.

Os chips conversores mais comuns usados ​​em placas ESP32-WROOM-32 estilo NodeMCU são:

* CP2102 – fabricado pela Silicon Labs.
* CH340G – fabricado pela WCH.

Você pode identificar o chip conversor examinando atentamente sua placa de desenvolvimento.

Cada um desses chips pode exigir um driver para funcionar corretamente com o sistema USB do seu computador.

Portanto, você deve identificar a ponte da sua placa.

1. Conecte a placa
2. Agora, abra um novo Terminal e execute: `dmesg | tail -n 20`
3. Adicionar usuário ao grupo de discagem (opcional, mas recomendado)
Para permitir acesso a **/dev/ttyUSB0** ou **/dev/ttyACM0** sem usar sudo, adicione seu usuário ao grupo dialout:
4. Depois, saia e faça login novamente (ou reinicie).
5. Verifique o dispositivo. Para verificar se o dispositivo está sendo reconhecido, execute o seguinte comando no terminal: `ls /dev/ttyUSB*` ou `ls /dev/ttyACM*`.
Esta é a porta serial que sua ferramenta de flash deve usar.

### Passar o binário para o microcontrolador usando o PlataforIO IDE
O PlatformIO IDE é uma extensão que transforma o Visual Studio Code em um ambiente de desenvolvimento integrado (IDE) profissional voltado para sistemas embarcados — como microcontroladores e placas IoT.

Pelo plataformIo é possível compilar o código e passar o binário para o microcontrolador via cabo usb.

#### micro-ros plataformio
Esta é uma biblioteca micro-ROS para projetos bare metal baseada no platformIO.

O processo de compilação para ROS 2 e micro-ROS é baseado em ferramentas personalizadas do sistema de meta-compilação e no CMake. O PlatformIO cuidará de todo o processo de compilação, incluindo dependências, compilação e vinculação.

1. Primeiro instale o ambiente virtual do python3: `sudo apt install python3-venv`.
2. Instale o CMake: `apt install -y git cmake python3-pip`.
3. Na aba de extensões do vs cod procure por **plataformIO IDE**, e instale.
4. Atualize o PlatformIO Core: `python3 -m pip install --upgrade platformio`.
Feche o vs cod e execute no terminal os comandos:

```shell
rm -rf ~/.platformio
rm -rf .pio
pio platform install espressif32
```

5. Em **PIO Home**, clique em **New Project**. 
  * Deve ser salvo em diretório que não possua espaço no nome (Area de trabalho por exemplo). 
  * O **framework** Arduino é usado para escrever códigos em arduino c++. Nosso caso.
  * O **framework** espidf é usado para escrever códigos em arduino c.
6. Apos criar o projeto, abra o arquivo **platformio.ini** para configurar de acordo com a documentação do [micro_ros_platformio](https://github.com/micro-ROS/micro_ros_platformio). Talvez seja necessário instalar o

```ini
[env:denky32]
platform = espressif32 @ ^6.12.0
board = denky32
framework = arduino

# Configurações
board_microros_distro = humble  # versão do ros2
board_microros_transport = wifi   # comunicação entre o micro-ros e o agente ros
lib_deps =
    https://github.com/micro-ROS/micro_ros_platformio

#No terminal do platformIO, no vs cod:
#    pio lib install # Install dependencies (execute apenas na primeira vez)
#    pio run # Build the firmware
#    pio run --target upload # Flash the firmware
#    pio run --target clean_microros  # Clean library
```

7. No arquivo **main.c** (para framework espidf), ou **main.cpp** (para o framwork Arduino) escreva o código que deseja compilar no microcontrolador.
8. Compile e passe para o micro processador.

Alguns comandos ulteis:
* Execute uma limpeza completa do build para garantir que nada antigo seja referenciado: `pio run --target clean_microros`
* compilar o projeto: `pio run`

#### Código main.cpp

```cpp
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <WiFi.h>
#include <IPAddress.h> // ALTERAÇÃO: Adicionado para clareza
#include <std_msgs/msg/string.h>

// ALTERAÇÃO: Use 'const char*' para strings literais para evitar warnings.
const char *ssid = "33robotics";
const char *password = "ponteaga";
const char *agent_ip_str = "192.168.1.139"; // ALTERAÇÃO: Renomeado para indicar que é uma string
uint32_t agent_port = 8888;

// ALTERAÇÃO: Objeto IPAddress para o agente
IPAddress agent_ip;

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
  Serial.print(agent_ip_str); // Imprime a string para visualização
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

  // ALTERAÇÃO: Converter a string do IP para o objeto IPAddress
  agent_ip.fromString(agent_ip_str);

  // micro-ROS transport & agent
  // ALTERAÇÃO: Passar o objeto IPAddress correto para a função
  set_microros_wifi_transports((char*)ssid, (char*)password, agent_ip, agent_port);
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
      data_buf;
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
  ping_msg.data.data = ping_data;
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

### Passar o binário para o microcontrolador com o **flash_tool**
Este método é usado para transferir os arquivos binários para o microcontrolador usando **flash_tool**, crie um arquivo python chamado **flash_tool.py**:

```python
#!/usr/bin/env python3
import sys
import glob
import os
import esptool
import serial
import time

# Change this to wherever you keep your firmware directories
BASE_FIRMWARE_DIR = os.getcwd()

#Esta função identifica todas as portas seriais disponíveis no sistema do usuário, dependendo do sistema operacional.
#No Linux, ela procura por /dev/ttyUSB*, no macOS, por /dev/tty.* e, no Windows, verifica de COM1 a COM256.
def list_serial_ports():
    """Return a list of available serial ports on Windows, macOS, or Linux."""
    if sys.platform.startswith('win'):
        ports = [f'COM{i+1}' for i in range(256)]
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')

    available = []
    for port in ports:
        try:
            with open(port):
                pass
            available.append(port)
        except Exception:
            continue
    return available

#Se houver apenas uma porta serial, ela será selecionada automaticamente. Caso contrário, o usuário será solicitado a escolher uma.
def choose_port():
    ports = list_serial_ports()
    if not ports:
        print("No serial ports found! Plug in your ESP32 and try again.")
        sys.exit(1)
    if len(ports) == 1:
        print(f"Auto-detected port: {ports[0]}")
        return ports[0]
    print("Available serial ports:")
    for idx, p in enumerate(ports, start=1):
        print(f"  {idx}: {p}")
    choice = input("Select port number: ")
    try:
        return ports[int(choice) - 1]
    except Exception:
        print("Invalid selection.")
        sys.exit(1)

#Esta função realiza o trabalho principal:
#Confirma que bootloader.bin, partitions.bin e firmware.bin existem dentro da pasta selecionada.
#Usa o módulo esptool para primeiro apagar a memória flash:
def flash(project_dir, port):
    """Erase flash and write bootloader, partition, and firmware bins."""
    boot  = os.path.join(project_dir, "bootloader.bin") 
    parts = os.path.join(project_dir, "partitions.bin") 
    app   = os.path.join(project_dir, "firmware.bin")

    for path in (boot, parts, app):
        if not os.path.isfile(path):
            print(f"Error: '{os.path.basename(path)}' not found in {project_dir}")
            sys.exit(1)

    print("Erasing flash…")
    esptool.main(['--chip','esp32','--port',port,'erase_flash'])

    #Em seguida, escreve os binários nos deslocamentos de memória padrão:
    print("Writing binaries:")
    print(f"  0x1000    {os.path.basename(boot)}")  # bootloader
    print(f"  0x8000    {os.path.basename(parts)}") # partition table
    print(f"  0x10000   {os.path.basename(app)}") # main firmware
    esptool.main([
        '--chip','esp32','--port',port,
        'write_flash',
        '0x1000',  boot,
        '0x8000',  parts,
        '0x10000', app
    ])
    print("Flash complete!")

# Após a atualização, este recurso opcional abre uma conexão serial na taxa de transmissão especificada e começa a imprimir a saída do ESP32 em tempo real. Ele aguarda 2 segundos após a abertura da porta, o que dá tempo para o ESP32 reiniciar.
#A saída é impressa até que você pressione Ctrl+C.
def serial_monitor(port, baudrate=115200):
    """Open a serial monitor on the specified port."""
    print(f"\nStarting serial monitor on {port} (baudrate {baudrate})")
    print("Press Ctrl+C to exit.\n")
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            time.sleep(2)  # Wait for ESP32 to reset after flash
            while True:
                if ser.in_waiting:
                    output = ser.read(ser.in_waiting).decode(errors='ignore')
                    print(output, end='', flush=True)
    except KeyboardInterrupt:
        print("\nSerial monitor stopped.")
    except Exception as e:
        print(f"Error: {e}")

def main():
    #A função main() controla tudo:
    #Sauda o usuário
    #Solicita o nome da pasta que contém os arquivos .bin
    #Detecta e confirma a porta serial
    #Chama flash()
    #Opcionalmente, inicia o monitor serial
    #Captura exceções e exibe erros úteis
    try:
        print("🔧 Welcome to the ESP32 Flasher Tool!")
        print("This tool flashes your ESP32 with bootloader, partitions, and firmware.\n")

        if len(sys.argv) == 2:
            project_name = sys.argv[1]
        else:
            project_name = input("Enter the name of the project folder (with .bin files): ").strip()

        project_dir = os.path.join(BASE_FIRMWARE_DIR, project_name)

        if not os.path.isdir(project_dir):
            print(f"❌ Error: project directory '{project_dir}' does not exist.")
            input("Press Enter to exit...")
            sys.exit(1)

        port = choose_port()
        flash(project_dir, port)

        answer = input("Launch serial monitor? [Y/n] (optionally: 'Y <baud_rate>'): ").strip().lower()
        if answer.startswith('y'):
            parts = answer.split()
            baudrate = 115200
            if len(parts) > 1:
                try:
                    baudrate = int(parts[1])
                except ValueError:
                    print("Invalid baud rate. Using default 115200.")
            serial_monitor(port, baudrate)

    except Exception as e:
        print(f"\n❌ An unexpected error occurred:\n{e}")
    finally:
        input("\nPress Enter to exit...")


if __name__ == "__main__":
    main()
```


Você pode executar este script diretamente de um terminal no seu computador se tiver o Python e as bibliotecas correspondentes instaladas, ou gerar um executável.

#### Micro-ROS Arduino 
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

#### Criando um cliente micro ros
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

#### Building the Client with Arduino CLI
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

O resultado do processo de **compilação** é um conjunto de arquivos binários que podem ser enviados para o ESP32.

Em seu ambiente de trabalho, você deverá conseguir ver os seguintes arquivos binários:

![U2_workspace_structure_microrospingnode](https://github.com/marcospontoexe/micro-ros/blob/main/imagens/U2_workspace_structure_microrospingnode.png)

Os mais importantes são:

1. **firmware.bin** (ou **.ino.bin**): Este é o seu aplicativo **micro-ROS compilado**. Ele contém seu código e todas as bibliotecas vinculadas.
2. **bootloader.bin**: Este é um pequeno binário que é executado primeiro quando o ESP32 é ligado. Ele inicializa o hardware, verifica a memória flash e carrega o aplicativo principal.
3. **partitions.bin**: Este define como a memória flash do ESP32 é particionada — para o aplicativo, para atualizações OTA, para armazenamento não volátil, etc.

Todos esses binários são necessários para uma implantação bem-sucedida. O bootloader do ESP32 espera arquivos específicos em deslocamentos de memória específicos e, se um estiver ausente ou posicionado incorretamente, a placa pode falhar na inicialização.

#### Baixe os binários

Agora, você deve baixar no seu computador host os binários necessários para atualizar a placa, crie uma nova pasta chamada microros_ping_node.

Dentro dela, baixe os arquivos binários renomeados com os nomes:

* bootloader.bin, para o bootloader.
* partitions.bin, para o arquivo de partições.
* firmware.bin, para o arquivo de firmware.

#### Executando o flash_tool
Conecte sua placa ESP32 usando um cabo de transferência de dados USB de boa qualidade. Certifique-se de que ele seja reconhecido corretamente pelo seu sistema.

Agora abra um terminal ou explorador de arquivos e navegue até a pasta onde você criou o python do flash_tool e do firmware (microros_ping_node). Execute o arquivo python pelo terminal: `python3 flash_tool.py`

Depois de executar os comandos acima, você verá:

```shell
Welcome to the ESP32 Flasher Tool!
This tool flashes your ESP32 with bootloader, partitions, and firmware.

Enter the name of the project folder (with .bin files):
```

Agora digite o nome da pasta do firmware que você deseja instalar. Por exemplo: **microros_ping_node**. Caso os binários estejam no mesmo diretório do flash_tool, basta pressionar enter.

A ferramenta detectará automaticamente sua porta serial. Se apenas uma for encontrada, ela prossegue. Se várias forem encontradas, ela mostra:

```shell
Available serial ports:
  1: COM3
  2: COM5
Select port number:
```

Lembre-se de pressionar o botão de inicialização por alguns segundos enquanto atualizamos o firmware!

Digite o número da porta conectada ao seu ESP32.

Em seguida, o flash será iniciado.

A maioria das placas de desenvolvimento ESP32 — incluindo a popular NodeMCU ESP32-WROOM-32 — vem com dois botões físicos identificados como:

* BOOT (às vezes identificado como IO0 ou FLASH)
* EN (abreviação de "Enable", às vezes identificado como RESET)

Esses botões não são programáveis ​​pelo usuário como os encontrados em um projeto Arduino. Em vez disso, eles são conectados diretamente aos pinos de controle do chip ESP32 e são usados ​​para controlar manualmente os modos de inicialização e resets.

O **botão EN** está conectado ao pino CHIP_PU (power-up) do ESP32. Ao pressioná-lo, você reinicia todo o ESP32, como se estivesse reiniciando um computador.

Este botão:

* Corta a energia momentaneamente para o chip ESP32
* Faz com que o microcontrolador reinicie do zero
* Se a flash contiver firmware válido, ele inicializará e funcionará normalmente

Você deve usá-lo para:

* Reiniciar o programa após a flash
* Reinicializar manualmente a placa caso ela pare de responder
* Às vezes, após conectar o monitor serial, para acionar a saída desde o início

Por outro lado, o **botão BOOT** está conectado ao pino GPIO0 e sua função é controlar como o ESP32 inicializa quando é ligado ou reiniciado. O GPIO0 é um dos poucos pinos de conexão — pinos especiais que o ESP32 lê durante a inicialização para determinar seu modo de operação.

Este botão:

Quando pressionado durante a reinicialização ou a inicialização, o ESP32 entra no modo de download de firmware (também conhecido como "modo flash" ou "modo bootloader UART").

Este modo permite que ferramentas externas como o esptool.py (usado dentro do seu flasher) carreguem binários pela conexão serial USB.

Você deve usá-lo:

* Ao colocar o ESP32 manualmente em modo flash
* Se a sua placa não entrar automaticamente no modo flash ou o esptool falhar com um tempo limite

Então, basicamente, quando o computador tentar flashear o ESP32, é muito importante que você pressione o botão de boot.

* Mantenha pressionado o botão BOOT
* Pressione e solte o botão EN
* Em seguida, solte o botão BOOT

Após uma atualização bem-sucedida, você deverá ver algo semelhante a isto:

```shell
Auto-detected port: COM4
Erasing flash…
esptool.py v4.8.1
Serial port COM4
Connecting..............
Chip is ESP32-D0WD-V3 (revision v3.1)
Features: WiFi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None
Crystal is 40MHz
MAC: 14:2b:2f:db:01:c0
Uploading stub...
Running stub...
Stub running...
Erasing flash (this may take a while)...
Chip erase completed successfully in 1.6s
Hard resetting via RTS pin...
Writing binaries:
  0x1000    bootloader.bin
  0x8000    partitions.bin
  0x10000   firmware.bin
esptool.py v4.8.1
Serial port COM4
Connecting.........
Chip is ESP32-D0WD-V3 (revision v3.1)
Features: WiFi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None
Crystal is 40MHz
MAC: 14:2b:2f:db:01:c0
Uploading stub...
Running stub...
Stub running...
Configuring flash size...
Flash will be erased from 0x00001000 to 0x00006fff...
Flash will be erased from 0x00008000 to 0x00008fff...
Flash will be erased from 0x00010000 to 0x000fafff...
Compressed 23488 bytes to 15080...
Wrote 23488 bytes (15080 compressed) at 0x00001000 in 1.5 seconds (effective 122.7 kbit/s)...
Hash of data verified.
Compressed 3072 bytes to 146...
Wrote 3072 bytes (146 compressed) at 0x00008000 in 0.1 seconds (effective 469.7 kbit/s)...
Hash of data verified.
Compressed 959040 bytes to 619448...
Wrote 959040 bytes (619448 compressed) at 0x00010000 in 56.9 seconds (effective 134.9 kbit/s)...
Hash of data verified.

Leaving...
Hard resetting via RTS pin...
Flash complete!
```

Após uma atualização bem-sucedida, você não só deverá ver o registro do terminal mostrado acima, como também o LED onboard do ESP32 deverá piscar, sinalizando que nosso programa foi iniciado.

Em seguida, ele perguntará se você deseja ou não iniciar o monitor serial.

```shell
Launch serial monitor? [Y/n] (optionally: 'Y '): Y
```

Pressione Y para monitorar a saída do ESP32 a 115200 bauds. Você deverá ver os logs de inicialização e o firmware em ação.

# Timers e Executores no micro ros
Um timer nos permite programar uma pequena função para ser executada a cada N milissegundos — sem bloquear o restante do sistema.

Pense em um timer como se fosse um lembrete:

"A cada 200 milissegundos, alternar o estado do LED."

Enquanto isso, o robo está livre para:

* Acionar motores
* Responder a mensagens /cmd_vel
* Processar outros tópicos do ROS

Vamos começar entendendo que tipo de mensagem usaremos para controlar os LEDs.

Usaremos uma mensagem padrão do ROS 2 String publicada no tópico /led_control, onde cada mensagem é um comando como:

