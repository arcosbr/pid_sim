# Simulação PID - Motor Hidráulico

## Descrição

Este projeto implementa uma **simulação física** de um pêndulo controlado por um motor hidráulico, utilizando um controlador PID (Proporcional, Integral e Derivativo) para estabilizar o sistema. A simulação inclui:

1. **Simulação Física**:
   - Modelagem do comportamento de um pêndulo sujeito à gravidade, atrito viscoso e torques externos.
   - Integração numérica utilizando métodos de Euler e Runge-Kutta de 4ª ordem.

2. **Controle PID**:
   - Ajuste dinâmico da pressão do motor hidráulico para manter o ângulo do pêndulo próximo ao valor desejado (setpoint).

3. **Interface Gráfica**:
   - Gráficos em tempo real que mostram os históricos de ângulo, erro e pressão.
   - Renderização visual do pêndulo para acompanhar o movimento do sistema.
   - Controles interativos para ajuste de parâmetros PID, pressão e reinicialização da simulação.

---

## Recursos

### Versão 1.0

1. **Simulação Física**:
   - Modelo realista para um pêndulo controlado por um motor hidráulico.
   - Integração numérica para simular o movimento ao longo do tempo.

2. **Controle PID**:
   - Controlador PID para manter o pêndulo na posição vertical.

3. **Interface Gráfica**:
   - Gráficos de ângulo e erro para monitorar o desempenho do controle.
   - Renderização visual do pêndulo para acompanhar o movimento.

4. **Parâmetros Ajustáveis**:
   - Ajuste da massa, comprimento, ganhos do PID e setpoint do pêndulo.

### Versão 1.1

1. **Estabilidade**:
   - Melhorias na estabilidade do sistema com ajustes nos ganhos do PID e na taxa de amostragem.

2. **Interface Gráfica**:
   - Inclusão de gráficos de erro e pressão para monitorar o desempenho do controle.
   - Controles interativos para ajustar os parâmetros PID em tempo real.

3. **Filtragem de Sinal**:
   - Aplicação de filtro exponencial para suavizar a pressão, reduzindo oscilações bruscas.

4. **Ruído no Sensor**:
   - Simulação de ruído nos sensores para adicionar realismo à medição do ângulo.

5. **Reinicialização**:
   - Opção para reiniciar a simulação a qualquer momento para testar diferentes configurações.

### Versão 1.2

1. **Limites Físicos**:
   - Adicionada saturação no torque do motor para refletir limitações reais de torque.

2. **Perturbação Externa**:
   - Parametrização da magnitude e direção da perturbação externa para maior controle sobre os testes.

3. **Ganhos do PID**:
   - Inclusão de limites nos ganhos \( K_p \), \( K_i \), \( K_d \) para evitar ajustes que possam desestabilizar o sistema.

4. **Validação Experimental**:
   - Comentários sobre a necessidade de validação experimental para garantir que os parâmetros correspondem ao sistema real.

5. **Documentação**:
   - Explicação detalhada da física do sistema, funcionamento do controle PID e implementação do código.

---

## Requisitos

Para compilar e executar esta simulação, você precisará dos seguintes componentes:

- **Compilador C**: GCC recomendado.
- **Raylib**: Biblioteca para criação de interfaces gráficas.
- **Make**: Opcional, para facilitar a compilação.

### Instalação do Raylib

#### Linux (Ubuntu/Debian)

```bash
sudo apt update
sudo apt install libraylib-dev
```

#### macOS

Usando Homebrew:

```bash
brew install raylib
```

#### Windows

1. Baixe e instale o [MinGW](http://www.mingw.org/).
2. Siga as instruções de instalação do Raylib para Windows disponíveis na [documentação oficial](https://www.raylib.com/).

---

## Compilação

### Passo a Passo

1. **Clone o Repositório**

   ```bash
   git clone https://github.com/seu-usuario/simulacao-pid-motor-hidraulico.git
   cd simulacao-pid-motor-hidraulico
   ```

2. **Compilar o Código**

   Utilize o seguinte comando para compilar o código com GCC:

   ```bash
   gcc -o simulacao main.c -lraylib -lm -lpthread -ldl -lrt -lX11
   ```

   **Notas:**
   - As opções de link (`-lraylib -lm -lpthread -ldl -lrt -lX11`) podem variar dependendo do sistema operacional.
   - No macOS, o comando pode ser simplificado:

     ```bash
     gcc -o simulacao main.c -lraylib -framework OpenGL -framework Cocoa -framework IOKit
     ```

3. **Executar a Simulação**

   ```bash
   ./simulacao
   ```

---

## Uso

Após compilar e executar a simulação, uma janela gráfica será aberta exibindo:

- **Gráficos em Tempo Real**:
  - **Ângulo (rad)**: Histórico do ângulo do pêndulo.
  - **Erro (rad)**: Histórico do erro entre o setpoint e o ângulo medido.
  - **Pressão (bar)**: Histórico da pressão aplicada pelo motor hidráulico.

- **Visualização do Pêndulo**:
  - Representação gráfica do pêndulo controlado, atualizada em tempo real.

- **Informações e Controles**:
  - Parâmetros atuais, como tempo de simulação, ângulo alvo, ângulo medido, pressão filtrada, duty cycle, torques, e ganhos do PID.

---

## Controles

Interaja com a simulação utilizando as seguintes teclas:

- **Ajuste de Pressão Manual**:
  - **Seta para Cima (`↑`)**: Aumenta a pressão manual em +1 bar.
  - **Seta para Baixo (`↓`)**: Diminui a pressão manual em -1 bar.

- **Ajuste de Ganhos PID**:
  - **F2**: Decrementa \( K_p \) em 10.
  - **F3**: Incrementa \( K_p \) em 10.
  - **F4**: Decrementa \( K_i \) em 0.01.
  - **F5**: Incrementa \( K_i \) em 0.01.
  - **F6**: Decrementa \( K_d \) em 10.
  - **F7**: Incrementa \( K_d \) em 10.

- **Método de Integração**:
  - **Tab**: Alterna entre os métodos de integração Euler e Runge-Kutta de 4ª ordem.

- **Perturbação Externa**:
  - **P**: Habilita/Desabilita a perturbação externa.
  - **J**: Aumenta a magnitude da perturbação externa.
  - **K**: Diminui a magnitude da perturbação externa.

- **Reinicialização da Simulação**:
  - **R**: Reseta a simulação para os valores iniciais.

---

## Validação Física

A simulação busca aderir fielmente à física do mundo real através dos seguintes componentes:

1. **Torque pela Gravidade**:
   - Calculado como \( T_g = -m \cdot g \cdot \frac{L}{2} \cdot \sin(\theta) \), representando o torque exercido pela gravidade no centro de massa do pêndulo.

2. **Torque pelo Atrito**:
   - Inclui atrito viscoso proporcional à velocidade angular (\( T_f = -c \cdot \omega \)).

3. **Torque do Motor**:
   - Calculado a partir da pressão do fluido hidráulico usando \( T_m = \frac{P \cdot D}{2 \pi} \cdot \text{fator de escala} \), adequado para motores hidráulicos.

4. **Momento de Inércia**:
   - Determinado como \( I = \frac{1}{3} \cdot m \cdot L^2 \), apropriado para uma barra uniforme pivotada em uma extremidade.

5. **Integração Numérica**:
   - Utiliza métodos padrão (Euler e RK4) com subdivisões temporais para maior estabilidade, apropriado para sistemas dinâmicos sensíveis.

6. **Filtragem de Sinal**:
   - Aplica um filtro exponencial para suavizar a pressão, reduzindo oscilações bruscas, prática em sistemas reais.

7. **Ruído no Sensor**:
   - Simula ruído nos sensores, adicionando realismo à medição do ângulo.

### Observação

A fidelidade da simulação depende da correspondência dos parâmetros (massa, atrito, deslocamento hidráulico, etc.) com o sistema real. Recomenda-se a realização de testes experimentais para validar e ajustar os parâmetros conforme necessário.

---

## Licença

Este projeto está licenciado sob a [Licença MIT](LICENSE), que permite uso comercial e não comercial. Consulte o arquivo `LICENSE` para obter mais informações.

---

## Contribuição

Contribuições são bem-vindas! Sinta-se à vontade para abrir issues ou enviar pull requests para melhorar este projeto.
