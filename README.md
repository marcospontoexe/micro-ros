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

### Agente Micro Ros
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

####  Executando o agente microROS
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

#### O agente IP
Para estabelecer um link de comunicação bem-sucedido entre o seu ESP32 e o agente micro-ROS em execução no seu computador, precisamos de duas informações essenciais:

* O **endereço IP do computador (host)** onde o agente está em execução.
* O **número da porta** onde o agente está escutando as mensagens micro-ROS recebidas.

Pense no seu endereço IP como o "endereço" do seu computador na sua rede local. Mas, assim como um prédio de apartamentos tem muitas portas e caixas de correio diferentes, seu computador tem muitos serviços em execução em portas diferentes. Um número de porta é como uma caixa de correio ou número de quarto específico — ele ajuda a encaminhar a mensagem para o aplicativo correto no seu computador.

Quando você conecta seu ESP32 ao Wi-Fi, ele se torna um dispositivo na sua rede local (LAN), assim como seu laptop ou desktop. Cada dispositivo conectado ao mesmo roteador recebe um endereço IP local exclusivo — normalmente algo como 192.168.0.24. Esses endereços permitem que os dispositivos "vejam" e se comuniquem entre si dentro da rede.

![U2_IP_adress](https://github.com/marcospontoexe/micro-ros/blob/main/imagens/U2_IP_adress.png)