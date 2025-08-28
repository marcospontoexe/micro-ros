# micro-ros
 Embora plataformas de computação de alto desempenho lidem com tarefas complexas de percepção e planejamento, é o hardware de baixo nível — microcontroladores, sensores e atuadores — que interage fisicamente com o mundo real.

Apesar de sua importância, a integração desses dispositivos embarcados com sistemas robóticos de nível superior tem sido tradicionalmente um processo fragmentado e difícil.

Diferentes linguagens de programação. Diferentes ferramentas de desenvolvimento. Diferentes protocolos de comunicação.

É aqui que o **micro-ROS** entra em ação.

O Micro-ROS preenche a lacuna entre o software de robótica moderno (ROS 2) e o desenvolvimento de sistemas embarcados.

Ele permite que você escreva nós ROS 2 que rodam em microcontroladores, dando ao seu código embarcado acesso nativo a objetos ROS: publicadores, assinantes, serviços e muito mais.

![U1_U2_micro-ROS_architecture.png](https://github.com/marcospontoexe/micro-ros/blob/main/imagens/U1_U2_micro-ROS_architecture.png).

Em vez de ter sistemas separados para "código robótico" e "firmware", o micro-ROS unifica o desenvolvimento em toda a pilha robótica — desde o planejamento de alto nível até as leituras de sensores de baixo nível.

Isso significa:

* Você pode enviar mensagens ROS 2 diretamente de sensores embarcados.
* Você pode controlar atuadores a partir de um gráfico ROS centralizado.
* Você pode estruturar seu robô como um sistema coeso e distribuído, onde até mesmo microcontroladores atuam como nós ROS 2 completos.
* Isso muda fundamentalmente a forma como os robôs são construídos.
* O Micro-ROS permite que você trate dispositivos embarcados como cidadãos de primeira classe dentro de uma arquitetura robótica — não mais caixas-pretas isoladas, mas participantes em tempo real de um ecossistema compartilhado.
