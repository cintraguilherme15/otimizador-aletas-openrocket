# Otimizador de Aletas para OpenRocket

Ferramenta em Python que automatiza o dimensionamento das aletas de foguetes projetados no OpenRocket.

## O problema
Ajustar as aletas manualmente no OpenRocket exige testar dezenas de combinações (envergadura, cordas e enflechamento) até chegar à estabilidade certa. Aletas pequenas deixam o foguete instável, e aletas grandes demais aumentam o arrasto e reduzem o apogeu.

## A solução
O otimizador testa automaticamente várias geometrias de aleta e retorna a que atinge a margem de estabilidade desejada com o menor arrasto possível, eliminando o processo de tentativa e erro manual.

## Como usar
1. Instale as dependências: `pip install -r requirements.txt`
2. Ajuste os parâmetros no código: faixas de dimensões das aletas e margem de estabilidade desejada
3. Execute: `python NOME_DO_ARQUIVO.py`

## Aplicação
Usado no desenvolvimento dos foguetes da EPTA, equipe de foguetemodelismo da UFU.

## Tecnologias
Python • OpenRocket

## Autor
**Rodrigo Olivo** – Membro Estruturas e Aerodinâmica na EPTA
## Coautor
**Guilherme Cintra Sousa** – Gerente Estruturas e Aerodinâmica na EPTA
[LinkedIn](https://www.linkedin.com/in/guilherme-cintra-sousa)

---
© 2026 Guilherme Cintra Sousa. Todos os direitos reservados. Disponibilizado apenas para visualização.
