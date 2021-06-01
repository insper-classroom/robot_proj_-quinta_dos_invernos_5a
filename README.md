# Projeto de Robótica Computacional

Grupo: **Quinta_dos_Invernos_5a**

Integrantes: 

* **Guilherme Rosada**
* **Ivan Barros**
* **Jamesson Santos**

_________

## Como funciona o projeto

Este projeto consiste na programação de um robô com uso do simulador Ros, com o objetivo de cumprir determinadas tarefas em um dado mapa. A missão do robô é percorrer uma pista em busca de um creeper de uma determinada cor, com um determinado identificador (QR Code) e, quando encontrado, o robô deve levá-lo 
até a estação (cubo com imagem de carro, vaca ou cachorro) percorrendo a pista.

### Lógica de funcionamento

Para a execução destas tarefas, utilizou-se uma máquina de estados que separa as ações do robô em cada uma das etapas, em que cada estado pode conter alguns subestados de controlam seu andamento do início ao fim. Sobre os principais estados:

- `trilhaON`: Esse estado é responsável por manter o robô percorrendo a pista.
- `searchTrilha`: Esse estado é acionado para fazer o robô procurar pela 
pista caso perca ela de vista.
- `arucoON`: Esse estado aciona o Aruco (leitor de QR Code) quando um creeper da 
cor desejada é encontrado, para que se possa identificar seu ID.
- `searchCreepON`: Esse estado ativa a busca pelo creeper em função da sua cor. 
Esse estado compreende vários subestados desde sua identificação até a sua captura.
- `retornarTrilha`: Procura a trilha no sentido em que o robô a percorria antes 
de sair da pista para capturar o creeper.
- `searchBaseON`: Esse estado ativa a rede neural que "lê" as imagens e identifica 
o conteúdo. É usada para identificar as bases (carro, cachorro, vaca ou cavalo).
- `checkpoint`: Estado que salva a direção do robô quando é acionado.

No início da execução, o robô recebe um *goal* que determina a cor do creeper, seu código e a base no qual o robô deve trabalhar. Os estados `trilhaON` e `searchCreepON` começam ativados, fazendo o robô percorrer a pista em busca do creeper da cor objetivo. Ao encontrar o creeper, o robô desativa `trilhaON` e faz a leitura do QR Code e, se for o creeper desejado, aciona a garra e aproxima-se dele até pegá-lo. Então, o robô volta para a pista no estado `retornarTrilha` e continua seu percurso, mas agora com o estado `searchBaseON` ativado, procurando pela base objetivo. Ao encontrá-la, ele centraliza com a base e leva o creeper até ela, controlando a garra para soltá-lo quando está muito próximo.

No estado `trilhaON`, a velocidade linear é controlada com base no ângulo de visão da faixa amarela em relação à vertical. Já a velocidade angular é controlada tanto pelo ângulo de visão quanto pelo erro calculado a partir da distância entre o centro da visão do robô e o centro médio dos contornos amarelos detectados.
