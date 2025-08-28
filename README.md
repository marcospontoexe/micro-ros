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