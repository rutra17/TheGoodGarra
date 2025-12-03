# 🦾 The Good 1A: Manipulador Robótico Autônomo

![Python](https://img.shields.io/badge/Python-3.8%2B-blue?style=for-the-badge\&logo=python)
![PyBullet](https://img.shields.io/badge/PyBullet-Simulation-orange?style=for-the-badge)
![Node-RED](https://img.shields.io/badge/Node--RED-Dashboard-red?style=for-the-badge\&logo=node-red)
![Status](https://img.shields.io/badge/Status-Concluído-success?style=for-the-badge)

> **Projeto de Manipulação Robótica e Supervisão (Robótica Computacional)** > *Controle cinemático avançado, máquina de estados e monitoramento em tempo real.*

---

## 📖 Sobre o Projeto

O **The Good 1A** é uma implementação de um sistema cyber-físico simulado onde um braço robótico **Kuka IIWA (7-DoF)** realiza a triagem autônoma de objetos. O sistema integra controle de baixo nível (física e cinemática) com supervisão de alto nível via **Node-RED**.

O diferencial deste projeto é o uso de **Cinemática Inversa com Null Space** (Espaço Nulo), garantindo que o robô mantenha posturas otimizadas (cotovelo elevado) enquanto executa tarefas de *Pick and Place*, evitando colisões e singularidades.

---

## 🚀 Funcionalidades Principais

### 🧠 Controle Inteligente

* **Null Space Inverse Kinematics:** Controle redundante de 7 eixos para movimentos fluidos e naturais.
* **Máquina de Estados Finita (FSM):** Lógica robusta para os estados `SEARCH`, `MOVE`, `GRAB`, `LIFT`, `TRANSPORT` e `DROP`.
* **Classificação Autônoma:** Identifica e separa **Cubos Verdes** 🟩 e **Esferas Azuis** 🔵, rejeitando distratores (Esferas Vermelhas 🔴).

### 📊 Supervisão via Node-RED

* **Dashboard em Tempo Real:** Monitoramento visual de métricas críticas.
* **Gráfico de Erro:** Visualização da convergência do efetuador (Estabilidade).
* **Monitor de Energia:** Estimativa de torque total aplicado nas juntas.
* **Controle Bidirecional:** Botão de **Start/Stop** remoto que atua como *Dead Man Switch* (segurança).

---

## 📂 Estrutura do Projeto

```text
Robo_Garra_1A/
├── 📂 src/
│   └── garra.py              # Código principal (Lógica de Controle e Física)
│
├── 📂 node_red/
│   └── dashboard_flow.json   # Fluxo de importação para o Dashboard
│
├── 📂 docs/
│   └── Relatorio_Tecnico.docx # Documentação detalhada do projeto
│
├── requirements.txt          # Lista de dependências Python
└── README.md                 # Este arquivo
```

---

## 🛠️ Instalação e Execução

### 1. Pré-requisitos

Certifique-se de ter instalado:

* [Python 3.x](https://www.python.org/)
* [Node-RED](https://nodered.org/)

### 2. Configuração do Ambiente

Clone o repositório e instale as dependências:

```bash
git clone https://github.com/SEU_USUARIO/TheGoodGarra.git
cd TheGoodGarra
pip install -r requirements.txt
```

### 3. Configurando o Node-RED

1. Inicie o Node-RED no terminal (`node-red`).
2. Acesse `http://localhost:1880`.
3. Vá em **Menu > Import** e selecione o arquivo `node_red/dashboard_flow.json`.
4. Clique em **Deploy** (Botão vermelho no topo).
5. Abra o Dashboard em `http://localhost:1880/ui`.

### 4. Rodando a Simulação

Com o Node-RED rodando, execute o script do robô:

```bash
python src/garra.py
```

> **Nota:** O robô iniciará no estado **PAUSADO**. Vá ao Dashboard e ative o interruptor **"ACIONAMENTO GERAL"** para iniciar a operação.

---

## 📊 Métricas Monitoradas

O sistema calcula e transmite as seguintes métricas a cada ciclo de supervisão:

1. **Erro de Posição (m):** Distância Euclidiana entre o *End-Effector* e o *Target*. Usado para validar a precisão do IK.
2. **Energia Estimada (Nm):** Somatório absoluto dos torques aplicados nas 7 juntas. Útil para identificar esforço excessivo ou carga pesada.
3. **Contagem de Itens:** Placar em tempo real da produtividade (Cubos vs. Esferas).

---

## 👨‍💻 Autor

Desenvolvido por **Artur**, **Bruno**, **Gabriel**, **Henrique** e **Yago** para a disciplina de Robótica Computacional.

* *Focado em Código Limpo, Arquitetura Modular e Integração IoT.*

---

## O que fazer agora?

1. **Edite o `README.md`** no seu computador e cole esse conteúdo.
2. **Imagens:** Note que no código eu apontei para `docs/print_simulacao.png` e `docs/print_dashboard.png`. Quando você tirar os prints para o relatório, salve cópias com esses nomes na pasta `docs/` e dê o `git push`. Assim, as imagens vão aparecer na página inicial do GitHub.
3. **Suba para o Git:**

```bash
git add .
git commit -m "Adicionando README Épico"
git push
```