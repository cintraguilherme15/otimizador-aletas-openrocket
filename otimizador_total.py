# OTIMIZADOR OPENROCKET 24.12 - V7 - 6VAR
# Gerado a partir de otimizador(1).py; alteracoes e teste descritos no LEIA-ME.
# Para executar, coloque este .py na mesma pasta do ORK, ENG e JAR.
# O foguete original nao e sobrescrito. Nao e garantia de seguranca real de voo.

##############################################################################################
#
##########                  Otimizador de Aletas Multifuncional                  #############                                                  
##########                  Equipe de Propulsão e Tecnologia Aeroespacial        #############
#
##############################################################################################




"""Versao V7 6VAR - OpenRocket 24.12.

Regras: vento operacional 10,8,6,4,2,0 m/s e seguranca 15 m/s.
Estabilidade >= 1.5 calibres na saida; estabilidade <= 3.5 no voo.
Objetivo: maximizar apogeu 0 m/s entre candidatos que passam nos 7 ventos.
Candidatos invalidos: registra menor violacao LOCAL (primeiro vento falho)
e maior apogeu 0 m/s; nao sao validos para construcao.
Otimiza raiz, ponta, altura, sweep, distancia boattail e comprimento ogiva.
Requer OpenRocket-24.12.jar, Simulação Merak Teórico.ork, Motor Merak J787.eng.
Dependencias: pip install JPype1 scipy.
"""

from __future__ import annotations

import csv
import math
import os
import shutil
import tempfile
import traceback
from pathlib import Path
from typing import Dict, List, Optional, Tuple

try:
    import jpype
    from jpype import JArray, JClass, JObject
except ImportError:
    print("ERRO: JPype1 nao esta instalado.")
    print("Instale com: python -m pip install JPype1")
    raise SystemExit(1)

try:
    from scipy.optimize import differential_evolution
except ImportError:
    print("ERRO: scipy nao esta instalado.")
    print("Instale com: python -m pip install scipy")
    raise SystemExit(1)


# ============================================================================
# 1. CONFIGURACAO - ALTERE AQUI QUANDO QUISER MUDAR O PROBLEMA
# ============================================================================

PASTA = Path(__file__).resolve().parent

ARQUIVO_JAR = PASTA / "OpenRocket-24.12.jar"
ARQUIVO_ORK = PASTA / "Simulação Merak Teórico.ork"
ARQUIVO_MOTOR = PASTA / "Motor Merak J787.eng"

# --------------------------------------------------------------------------
# Ventos
# --------------------------------------------------------------------------

# Ordem proposital: comecar pela condicao operacional mais severa.
VENTOS_OPERACIONAIS = [10.0, 8.0, 6.0, 4.0, 2.0, 0.0]

# Condicao de fator de seguranca, testada somente depois da faixa operacional.
VENTO_SEGURANCA = 15.0

# --------------------------------------------------------------------------
# Regras de estabilidade
# --------------------------------------------------------------------------

# Em QUALQUER vento testado, a estabilidade na saida da base/trilho
# deve ser pelo menos 1.5 cal.
ESTABILIDADE_MINIMA_SAIDA = 1.5

# Em QUALQUER vento testado, a estabilidade deve permanecer abaixo deste
# valor durante TODO o historico de voo retornado pelo OpenRocket.
ESTABILIDADE_MAXIMA = 3.5

# Se a equipe decidir trabalhar com 3.8 como limite:
# ESTABILIDADE_MAXIMA = 3.8

# --------------------------------------------------------------------------
# Limites geometricos - CENTIMETROS
# --------------------------------------------------------------------------
#
# Estes sao os limites da BUSCA, nao as dimensoes do foguete atual.
# Edite-os para refletir o que a equipe realmente consegue fabricar.
#
# sweep = deslocamento longitudinal entre a raiz e a ponta da aleta.
#
# distancia_boattail = distancia POSITIVA entre o bordo de fuga da raiz
# da aleta e o inicio do boattail.
#     0 cm  -> a raiz termina exatamente onde o boattail comeca.
#     3 cm  -> ficam 3 cm de fuselagem entre a raiz e o boattail.
#
# ATENCAO: os limites de distancia_boattail e comprimento_ogiva abaixo
# sao configuracoes iniciais; confirme-os com a equipe antes da busca final.
# A ogiva atual do arquivo possui aproximadamente 16 cm.
#
LIMITES_CM = {
    "corda_raiz": (10.0, 30.0),
    "corda_ponta": (10.0, 30.0),
    "altura": (8.0, 14.0),
    "sweep": (6.0, 8.0),
    "distancia_boattail": (0.0, 2.0),
    "comprimento_ogiva": (16.0, 16.0),
}

# Opcional: se True, rejeita qualquer candidato com ponta > raiz
# antes mesmo de abrir uma simulacao.
EXIGIR_PONTA_MENOR_IGUAL_RAIZ = False

# --------------------------------------------------------------------------
# Modelo de massa das aletas
# --------------------------------------------------------------------------
# Opcoes:
#   "arquivo_ork"        -> preserva o que esta salvo no .ork.
#   "material_openrocket"-> remove override de massa e usa material/espessura.
#   "massa_areal"        -> usa kg/m2 medido na chapa real.
#   "massa_total_fixa"   -> usa uma massa total fixa para todo o conjunto.
#
# Recomendada para a chapa real da equipe: "massa_areal".
# Mantemos "arquivo_ork" por padrao porque a massa areal real ainda nao
# foi informada; assim nao inventamos um valor fisico.
MODO_MASSA_ALETAS = "arquivo_ork"

# Preencha antes de selecionar MODO_MASSA_ALETAS = "massa_areal".
# Exemplo de calculo:
# massa_areal = massa_da_amostra_kg / area_da_amostra_m2
MASSA_AREAL_ALETAS_KG_M2 = None

# Massa adicional fixa do conjunto (cola, reforcos, insertos etc.), se quiser
# incorpora-la ao modelo de massa areal. Deixe 0.0 para ignorar.
MASSA_EXTRA_ALETAS_KG = 0.0

# Preencha antes de selecionar MODO_MASSA_ALETAS = "massa_total_fixa".
MASSA_TOTAL_FIXA_ALETAS_KG = None

# --------------------------------------------------------------------------
# OpenRocket
# --------------------------------------------------------------------------

# Nomes preferidos no .ork. Se nao encontrar, o codigo tenta um fallback
# pelo tipo de componente.
NOME_ALETA_PREFERIDO = "Aletas"
NOME_OGIVA_PREFERIDO = "Ogiva"
NOME_BOATTAIL_PREFERIDO = "Boattail"

# Simulacao salva no .ork usada como MODELO das demais configuracoes:
# motor, trilho, atmosfera, local de lancamento etc.
# 0 = primeira simulacao do arquivo.
INDICE_SIMULACAO_BASE = 0

# Zera turbulencia para que candidatos diferentes sejam comparados sob
# exatamente as mesmas condicoes.
DESVIO_PADRAO_VENTO = 0.0
# Quando True, nao aprova uma simulacao com estabilidade indefinida
# (NaN/inf) desde a saida do trilho. Se o OpenRocket retornar NaN durante
# a descida, investigue a causa antes de desativar esta verificacao.
REJEITAR_NAO_FINITA_APOS_SAIDA = True
SEMENTE_ALEATORIA = 123456

# --------------------------------------------------------------------------
# Differential Evolution
# --------------------------------------------------------------------------

# Com 6 variaveis, o tamanho nominal da populacao e aproximadamente:
#     POPSIZE * 6
# Ex.: POPSIZE=20 -> aproximadamente 120 candidatos por geracao.
# O total maximo aproximado e:
#     populacao_inicial + MAXITER * populacao
MAXITER = 35
POPSIZE = 12
TOL = 0.005
SEMENTE_OTIMIZADOR = 2026

# --------------------------------------------------------------------------
# Arquivos de saida
# --------------------------------------------------------------------------

ARQUIVO_HISTORICO = PASTA / "historico_otimizacao_6var.csv"
ARQUIVO_RESULTADO = PASTA / "resultado_otimizacao_6var.csv"
ARQUIVO_ORK_OTIMIZADO = PASTA / "foguete_otimizado_6var.ork"
ARQUIVO_MELHORES_INVALIDOS = PASTA / "melhores_invalidos_6var.csv"

# Mostra cada candidato no terminal.
IMPRIMIR_TODAS_AVALIACOES = True


# ============================================================================
# 2. INICIALIZACAO DO OPENROCKET
# ============================================================================

def encontrar_jvm() -> str:
    """Encontra a JVM instalada no sistema."""
    try:
        return jpype.getDefaultJVMPath()
    except Exception:
        candidatos = []

        java_home = os.environ.get("JAVA_HOME")
        if java_home:
            candidatos.extend([
                Path(java_home) / "bin" / "server" / "jvm.dll",
                Path(java_home) / "lib" / "server" / "libjvm.so",
            ])

        candidatos.extend([
            Path(r"C:\Program Files\OpenRocket\jre\bin\server\jvm.dll"),
            Path(r"C:\Program Files (x86)\OpenRocket\jre\bin\server\jvm.dll"),
        ])

        for caminho in candidatos:
            if caminho.exists():
                return str(caminho)

        raise RuntimeError(
            "Nao foi possivel localizar a JVM. "
            "Instale Java ou configure JAVA_HOME."
        )


def preparar_jar_em_caminho_seguro() -> Path:
    """
    Copia o JAR para uma pasta temporaria sem acentos.

    Isso evita o problema observado no Windows quando o caminho continha
    'Area de Trabalho' com caractere acentuado.
    """
    pasta_temp = Path(tempfile.gettempdir()) / "openrocket_optimizer_24_12"
    pasta_temp.mkdir(parents=True, exist_ok=True)

    jar_seguro = pasta_temp / "OpenRocket-24.12.jar"

    precisa_copiar = True

    if jar_seguro.exists():
        try:
            precisa_copiar = (
                jar_seguro.stat().st_size != ARQUIVO_JAR.stat().st_size
                or jar_seguro.stat().st_mtime < ARQUIVO_JAR.stat().st_mtime
            )
        except OSError:
            precisa_copiar = True

    if precisa_copiar:
        shutil.copy2(ARQUIVO_JAR, jar_seguro)

    return jar_seguro


def carregar_motor_eng_na_base() -> None:
    """Registra explicitamente o J787 do arquivo .eng na base do OpenRocket."""
    GeneralMotorLoader = JClass(
        "info.openrocket.core.file.motor.GeneralMotorLoader"
    )
    FileInputStream = JClass("java.io.FileInputStream")
    Application = JClass("info.openrocket.core.startup.Application")

    stream = FileInputStream(str(ARQUIVO_MOTOR))

    try:
        builders = GeneralMotorLoader().load(stream, ARQUIVO_MOTOR.name)
    finally:
        stream.close()

    if builders is None or int(builders.size()) == 0:
        raise RuntimeError(
            f"Nenhuma curva de motor encontrada em '{ARQUIVO_MOTOR.name}'."
        )

    banco = Application.getThrustCurveMotorSetDatabase()
    nomes = []

    for builder in builders:
        motor = builder.build()
        banco.addMotor(motor)

        try:
            fabricante = str(motor.getManufacturer().getDisplayName())
        except Exception:
            fabricante = str(motor.getManufacturer())

        nomes.append(f"{fabricante} {motor.getDesignation()}")

    print("Motor .eng carregado: " + ", ".join(nomes))


def iniciar_openrocket() -> None:
    """Inicia JVM, OpenRocket e bancos internos."""
    for caminho, descricao in (
        (ARQUIVO_JAR, "JAR"),
        (ARQUIVO_ORK, "ORK"),
        (ARQUIVO_MOTOR, "ENG"),
    ):
        if not caminho.exists():
            raise FileNotFoundError(
                f"{descricao} nao encontrado: {caminho}"
            )

    jar_seguro = preparar_jar_em_caminho_seguro()

    if not jpype.isJVMStarted():
        jpype.startJVM(
            encontrar_jvm(),
            "-ea",
            "-Djava.awt.headless=true",
            classpath=[str(jar_seguro)],
        )

    # Reduz logs desnecessarios.
    try:
        LoggerFactory = JClass("org.slf4j.LoggerFactory")
        LogbackLogger = JClass("ch.qos.logback.classic.Logger")
        Level = JClass("ch.qos.logback.classic.Level")

        root_logger = LoggerFactory.getLogger("ROOT")
        JObject(root_logger, LogbackLogger).setLevel(Level.ERROR)
    except Exception:
        pass

    CoreModule = JClass("info.openrocket.core.startup.CoreModule")
    PluginModule = JClass("info.openrocket.core.plugin.PluginModule")
    Application = JClass("info.openrocket.core.startup.Application")
    Guice = JClass("com.google.inject.Guice")
    Module = JClass("com.google.inject.Module")

    core_module = CoreModule()
    plugin_module = PluginModule()

    modules = JArray(Module)([core_module, plugin_module])
    injector = Guice.createInjector(modules)

    Application.setInjector(injector)

    core_module.startLoader()

    for nome_campo in ("motorLoader", "presetLoader"):
        campo = CoreModule.class_.getDeclaredField(nome_campo)
        campo.setAccessible(True)
        carregador = campo.get(core_module)
        carregador.blockUntilLoaded()

    # O custom motor precisa existir ANTES de abrir o .ork.
    carregar_motor_eng_na_base()


# ============================================================================
# 3. FOGUETE / GEOMETRIA
# ============================================================================

def carregar_documento():
    GeneralRocketLoader = JClass(
        "info.openrocket.core.file.GeneralRocketLoader"
    )
    JavaFile = JClass("java.io.File")

    loader = GeneralRocketLoader(JavaFile(str(ARQUIVO_ORK)))
    return loader.load()


def percorrer_componentes(componente):
    yield componente

    for filho in componente.getChildren():
        yield from percorrer_componentes(filho)


def encontrar_aleta(foguete):
    TrapezoidFinSet = JClass(
        "info.openrocket.core.rocketcomponent.TrapezoidFinSet"
    )

    candidatas = []

    for componente in percorrer_componentes(foguete):
        if TrapezoidFinSet.class_.isInstance(componente):
            candidatas.append(componente)

            if str(componente.getName()) == NOME_ALETA_PREFERIDO:
                return componente

    if not candidatas:
        raise RuntimeError(
            "Nenhum conjunto de aletas trapezoidais foi encontrado no .ork."
        )

    print(
        f"AVISO: aleta '{NOME_ALETA_PREFERIDO}' nao encontrada. "
        f"Usando '{candidatas[0].getName()}'."
    )

    return candidatas[0]


def encontrar_ogiva(foguete):
    NoseCone = JClass(
        "info.openrocket.core.rocketcomponent.NoseCone"
    )

    candidatas = []

    for componente in percorrer_componentes(foguete):
        if NoseCone.class_.isInstance(componente):
            candidatas.append(componente)

            if str(componente.getName()) == NOME_OGIVA_PREFERIDO:
                return componente

    if not candidatas:
        raise RuntimeError("Nenhuma ogiva (NoseCone) foi encontrada no .ork.")

    print(
        f"AVISO: ogiva '{NOME_OGIVA_PREFERIDO}' nao encontrada. "
        f"Usando '{candidatas[0].getName()}'."
    )

    return candidatas[0]


def encontrar_boattail(foguete):
    Transition = JClass(
        "info.openrocket.core.rocketcomponent.Transition"
    )
    NoseCone = JClass(
        "info.openrocket.core.rocketcomponent.NoseCone"
    )

    transicoes = []

    for componente in percorrer_componentes(foguete):
        if (
            Transition.class_.isInstance(componente)
            and not NoseCone.class_.isInstance(componente)
        ):
            transicoes.append(componente)

            if str(componente.getName()) == NOME_BOATTAIL_PREFERIDO:
                return componente

    if not transicoes:
        raise RuntimeError(
            "Nenhuma transicao candidata a boattail foi encontrada no .ork."
        )

    print(
        f"AVISO: boattail '{NOME_BOATTAIL_PREFERIDO}' nao encontrado. "
        f"Usando a ultima transicao: '{transicoes[-1].getName()}'."
    )

    return transicoes[-1]


def posicao_absoluta_frente(componente) -> float:
    """
    Retorna a coordenada axial absoluta da frente do componente, em m.

    No OpenRocket 24.12, alguns objetos chegam ao JPype com o tipo
    estatico AxialPositionable. Essa interface so expoe getAxialOffset()
    sem argumentos, embora RocketComponent possua a sobrecarga
    getAxialOffset(AxialMethod). Para evitar erro de despacho de
    sobrecarga no JPype, usamos diretamente getComponentLocations()[0].x.

    O proprio RocketComponent.getAxialOffset(ABSOLUTE) usa exatamente
    essa coordenada internamente.
    """
    RocketComponent = JClass(
        "info.openrocket.core.rocketcomponent.RocketComponent"
    )

    componente_rc = JObject(componente, RocketComponent)
    localizacoes = componente_rc.getComponentLocations()

    if localizacoes is None or len(localizacoes) == 0:
        raise RuntimeError(
            f"Nao foi possivel obter a posicao absoluta de "
            f"'{componente.getName()}'."
        )

    return float(localizacoes[0].x)


def distancia_aleta_boattail(aleta, boattail) -> float:
    """
    Distancia axial entre o bordo de fuga da RAIZ da aleta e a frente
    do boattail.

    Valor positivo = existe um espaco de fuselagem entre ambos.
    Valor zero     = a raiz termina exatamente no inicio do boattail.
    Valor negativo = sobreposicao axial.
    """
    frente_aleta = posicao_absoluta_frente(aleta)
    bordo_fuga_raiz = frente_aleta + float(aleta.getRootChord())
    frente_boattail = posicao_absoluta_frente(boattail)
    return frente_boattail - bordo_fuga_raiz


def geometria_atual(aleta, ogiva, boattail) -> Tuple[float, ...]:
    """
    Retorna, em metros:
      raiz, ponta, altura, sweep, distancia_boattail, comprimento_ogiva
    """
    return (
        float(aleta.getRootChord()),
        float(aleta.getTipChord()),
        float(aleta.getHeight()),
        float(aleta.getSweep()),
        float(distancia_aleta_boattail(aleta, boattail)),
        float(ogiva.getLength()),
    )


def area_aleta_individual_m2(x) -> float:
    raiz = float(x[0])
    ponta = float(x[1])
    altura = float(x[2])
    return 0.5 * (raiz + ponta) * altura


def massa_aletas_por_massa_areal_kg(aleta, x) -> float:
    if MASSA_AREAL_ALETAS_KG_M2 is None:
        raise RuntimeError(
            "MODO_MASSA_ALETAS='massa_areal', mas "
            "MASSA_AREAL_ALETAS_KG_M2 nao foi preenchida."
        )

    massa_areal = float(MASSA_AREAL_ALETAS_KG_M2)

    if massa_areal <= 0.0:
        raise RuntimeError("MASSA_AREAL_ALETAS_KG_M2 deve ser > 0.")

    n_aletas = int(aleta.getFinCount())
    area_total = area_aleta_individual_m2(x) * n_aletas

    return (
        area_total * massa_areal
        + float(MASSA_EXTRA_ALETAS_KG)
    )


def aplicar_modelo_massa_aletas(aleta, x) -> None:
    modo = str(MODO_MASSA_ALETAS).strip().lower()

    if modo == "arquivo_ork":
        # Nao altera a configuracao de massa originalmente carregada.
        return

    if modo == "material_openrocket":
        # Faz o OpenRocket voltar a calcular a massa a partir de material,
        # espessura e geometria.
        aleta.setMassOverridden(False)
        return

    if modo == "massa_areal":
        massa = massa_aletas_por_massa_areal_kg(aleta, x)
        aleta.setOverrideMass(float(massa))
        aleta.setMassOverridden(True)
        return

    if modo == "massa_total_fixa":
        if MASSA_TOTAL_FIXA_ALETAS_KG is None:
            raise RuntimeError(
                "MODO_MASSA_ALETAS='massa_total_fixa', mas "
                "MASSA_TOTAL_FIXA_ALETAS_KG nao foi preenchida."
            )

        massa = float(MASSA_TOTAL_FIXA_ALETAS_KG)

        if massa <= 0.0:
            raise RuntimeError("MASSA_TOTAL_FIXA_ALETAS_KG deve ser > 0.")

        aleta.setOverrideMass(massa)
        aleta.setMassOverridden(True)
        return

    raise RuntimeError(
        "MODO_MASSA_ALETAS invalido. Use: arquivo_ork, "
        "material_openrocket, massa_areal ou massa_total_fixa."
    )


def massa_aletas_atual_kg(aleta) -> float:
    """Massa total do componente FinSet segundo o OpenRocket, em kg."""
    return float(aleta.getMass())


def aplicar_geometria(foguete, aleta, ogiva, boattail, x) -> None:
    """
    Aplica as 6 variaveis do candidato:
      raiz, ponta, altura, sweep, distancia_boattail, comprimento_ogiva.

    A posicao da aleta e definida em relacao a frente REAL do boattail,
    portanto continua coerente mesmo quando o comprimento da ogiva muda.
    """
    (
        raiz,
        ponta,
        altura,
        sweep,
        distancia_boattail_m,
        comprimento_ogiva_m,
    ) = map(float, x)

    AxialMethod = JClass(
        "info.openrocket.core.rocketcomponent.position.AxialMethod"
    )

    # 1) Muda comprimentos/formato. Apos update(), componentes que usam
    # posicionamento sequencial sao reposicionados pelo OpenRocket.
    foguete.freeze()
    try:
        ogiva.setLength(comprimento_ogiva_m)
        aleta.setRootChord(raiz)
        aleta.setTipChord(ponta)
        aleta.setHeight(altura)
        aleta.setSweep(sweep)
    finally:
        foguete.thaw()

    foguete.update()

    # 2) Posiciona a raiz da aleta a uma distancia especificada da frente
    # do boattail. O offset ABSOLUTE refere-se a frente da aleta.
    frente_boattail = posicao_absoluta_frente(boattail)
    frente_aleta_desejada = (
        frente_boattail
        - distancia_boattail_m
        - raiz
    )

    foguete.freeze()
    try:
        aleta.setAxialMethod(AxialMethod.ABSOLUTE)
        aleta.setAxialOffset(float(frente_aleta_desejada))
        aplicar_modelo_massa_aletas(aleta, x)
    finally:
        foguete.thaw()

    foguete.update()

# ============================================================================
# 4. SIMULACAO
# ============================================================================

def criar_simulacao_temporaria(
    documento,
    foguete,
    simulacao_base,
    vento: float,
):
    """
    Cria uma simulacao nova a partir da configuracao-base.

    A simulacao salva no arquivo original nao e' sobrescrita.
    """
    Simulation = JClass("info.openrocket.core.document.Simulation")
    WindModelType = JClass(
        "info.openrocket.core.models.wind.WindModelType"
    )

    sim = Simulation(documento, foguete)

    sim.setFlightConfigurationId(
        simulacao_base.getFlightConfigurationId()
    )

    sim.copySimulationOptionsFrom(
        simulacao_base.getOptions()
    )

    try:
        sim.copyExtensionsFrom(
            simulacao_base.getSimulationExtensions()
        )
    except Exception:
        pass

    opcoes = sim.getOptions()

    opcoes.setWindModelType(WindModelType.AVERAGE)
    opcoes.setWindSpeedAverage(float(vento))
    opcoes.setWindSpeedDeviation(float(DESVIO_PADRAO_VENTO))
    opcoes.setRandomSeed(int(SEMENTE_ALEATORIA))

    sim.setName(f"Otimizador - vento {vento:g} m/s")

    return sim


def lista_java_para_float(lista) -> List[float]:
    if lista is None:
        return []

    return [
        float(lista.get(i))
        for i in range(int(lista.size()))
    ]


def avisos_da_simulacao(simulacao) -> str:
    """Retorna warnings do OpenRocket em uma unica string curta."""
    try:
        warnings = simulacao.getSimulatedWarnings()

        if warnings is None or int(warnings.size()) == 0:
            return ""

        textos = []

        for warning in warnings:
            texto = str(warning).replace("\n", " ").strip()
            if texto and texto not in textos:
                textos.append(texto)

        return " | ".join(textos)

    except Exception:
        return ""


def verificar_abortos(ramo) -> Optional[str]:
    """
    Rejeita apenas abortos/erros reais da simulacao.

    Ausencia do evento APOGEE NAO e' considerada erro nesta versao.
    """
    FlightEventType = JClass(
        "info.openrocket.core.simulation.FlightEvent$Type"
    )

    for tipo, nome in (
        (FlightEventType.SIM_ABORT, "SIM_ABORT"),
        (FlightEventType.EXCEPTION, "EXCEPTION"),
    ):
        evento = ramo.getFirstEvent(tipo)

        if evento is not None:
            return f"OpenRocket registrou {nome} em t={float(evento.getTime()):.3f} s"

    return None


def interpolar_no_evento(tempos, valores, t_evento: float) -> float:
    """Interpola entre amostras adjacentes, sem extrapolar nem pular NaN/inf.

    O evento LAUNCHROD pode ocorrer entre dois registros da serie temporal.
    """
    if len(tempos) != len(valores) or not tempos:
        raise RuntimeError("Series temporais inconsistentes para interpolacao.")
    if not math.isfinite(t_evento):
        raise RuntimeError("Tempo do evento LAUNCHROD invalido.")
    tolerancia = 1e-9
    for i, t in enumerate(tempos):
        if not math.isfinite(t):
            raise RuntimeError("TYPE_TIME contem tempo nao finito.")
        if abs(t - t_evento) <= tolerancia:
            if math.isfinite(valores[i]):
                return float(valores[i])
            raise RuntimeError("Estabilidade nao finita exatamente na saida do trilho.")
    for i in range(len(tempos)-1):
        t0, t1 = tempos[i], tempos[i+1]
        if t0 < t_evento < t1:
            v0, v1 = valores[i], valores[i+1]
            if not (math.isfinite(v0) and math.isfinite(v1)):
                raise RuntimeError("Estabilidade indefinida no intervalo da saida do trilho.")
            return float(v0 + (v1-v0)*(t_evento-t0)/(t1-t0))
    raise RuntimeError("Evento LAUNCHROD fora do intervalo dos dados temporais.")


def extrair_metricas(simulacao, vento: float) -> Dict[str, float | str]:
    """Extrai apogeu, estabilidade na saida interpolada e maxima do historico.

    Rejeita estabilidade indefinida a partir do evento LAUNCHROD por padrao;
    um resultado que nao pode ser verificado nao deve ser tratado como valido.
    """
    FlightDataType = JClass("info.openrocket.core.simulation.FlightDataType")
    FlightEventType = JClass("info.openrocket.core.simulation.FlightEvent$Type")
    dados = simulacao.getSimulatedData()
    if dados is None or int(dados.getBranchCount()) == 0:
        raise RuntimeError("A simulacao nao retornou dados de voo.")
    if int(dados.getBranchCount()) != 1:
        raise RuntimeError(
            "Simulacao retornou varios ramos/estagios: verificar manualmente "
            "a regra de estabilidade antes de usar esta versao."
        )
    ramo = dados.getBranch(0)
    aborto = verificar_abortos(ramo)
    if aborto:
        avisos = avisos_da_simulacao(simulacao)
        raise RuntimeError(aborto + (" | Avisos: " + avisos if avisos else ""))
    evento = ramo.getFirstEvent(FlightEventType.LAUNCHROD)
    if evento is None:
        raise RuntimeError("Evento LAUNCHROD nao encontrado: o foguete nao saiu do trilho.")
    tempos = lista_java_para_float(ramo.get(FlightDataType.TYPE_TIME))
    altitudes = lista_java_para_float(ramo.get(FlightDataType.TYPE_ALTITUDE))
    estabilidades = lista_java_para_float(ramo.get(FlightDataType.TYPE_STABILITY))
    if not tempos or not altitudes or not estabilidades:
        raise RuntimeError("Dados TIME, ALTITUDE ou STABILITY ausentes.")
    if not (len(tempos) == len(altitudes) == len(estabilidades)):
        raise RuntimeError("Comprimentos dos historicos de voo nao coincidem.")
    if any(not math.isfinite(t) for t in tempos):
        raise RuntimeError("Historico de tempo contem NaN/inf.")
    if any(t1 < t0 for t0, t1 in zip(tempos, tempos[1:])):
        raise RuntimeError("Historico de tempo nao esta em ordem cronologica.")
    tempo_saida = float(evento.getTime())
    s_saida = interpolar_no_evento(tempos, estabilidades, tempo_saida)
    # Analisa todas as amostras finitas do historico para S_max.
    valores_validos = [s for s in estabilidades if math.isfinite(s)]
    if not valores_validos:
        raise RuntimeError("Nao ha estabilidade finita em nenhum trecho do voo.")
    if REJEITAR_NAO_FINITA_APOS_SAIDA:
        indices_indefinidos = [i for i, (t, s) in enumerate(zip(tempos, estabilidades))
                               if t >= tempo_saida - 1e-9 and not math.isfinite(s)]
        if indices_indefinidos:
            raise RuntimeError(
                f"Estabilidade nao finita apos saida do trilho "
                f"({len(indices_indefinidos)} amostras, "
                f"primeira em t={tempos[indices_indefinidos[0]]:.4f} s)."
            )
    est_max = max(valores_validos)
    indices_altitude = [i for i, h in enumerate(altitudes) if math.isfinite(h)]
    if not indices_altitude:
        raise RuntimeError("Altitude do voo totalmente indefinida.")
    if any(t >= tempo_saida and not math.isfinite(h) for t, h in zip(tempos, altitudes)):
        raise RuntimeError("Altitude nao finita apos saida do trilho.")
    i_apogeu = max(indices_altitude, key=lambda i: altitudes[i])
    if tempos[i_apogeu] <= tempo_saida:
        raise RuntimeError("Altitude maxima antes ou na saida do trilho.")
    apogeu = float(altitudes[i_apogeu])
    if not math.isfinite(apogeu):
        raise RuntimeError("Apogeu nao finito.")
    try:
        velocidade_saida = float(dados.getLaunchRodVelocity())
    except Exception:
        velocidades = lista_java_para_float(ramo.get(FlightDataType.TYPE_VELOCITY_TOTAL))
        velocidade_saida = (
            interpolar_no_evento(tempos, velocidades, tempo_saida)
            if len(velocidades) == len(tempos) else float('nan')
        )
    return {
        "vento": float(vento),
        "estabilidade_saida": s_saida,
        "estabilidade_maxima": float(est_max),
        "velocidade_saida": float(velocidade_saida),
        "apogeu": apogeu,
        "tempo_saida": tempo_saida,
        "tempo_apogeu": float(tempos[i_apogeu]),
        "avisos": avisos_da_simulacao(simulacao),
    }


def simular_vento(
    documento,
    foguete,
    simulacao_base,
    vento: float,
) -> Dict[str, float | str]:
    """Executa uma unica condicao de vento."""
    sim = criar_simulacao_temporaria(
        documento,
        foguete,
        simulacao_base,
        vento,
    )

    try:
        sim.simulate()
    except Exception as exc:
        avisos = avisos_da_simulacao(sim)

        mensagem = (
            f"falha da simulacao em {vento:g} m/s: {exc}"
        )

        if avisos:
            mensagem += f" | Avisos: {avisos}"

        raise RuntimeError(mensagem) from exc

    return extrair_metricas(sim, vento)


# ============================================================================
# 5. REGRAS DO CANDIDATO
# ============================================================================

def cm(valor_m: float) -> float:
    return float(valor_m) * 100.0


def area_aleta_m2(x) -> float:
    """Area planform de UMA aleta, em m2."""
    return area_aleta_individual_m2(x)


def limites_em_metros():
    ordem = (
        "corda_raiz",
        "corda_ponta",
        "altura",
        "sweep",
        "distancia_boattail",
        "comprimento_ogiva",
    )

    return [
        (
            LIMITES_CM[nome][0] / 100.0,
            LIMITES_CM[nome][1] / 100.0,
        )
        for nome in ordem
    ]


def fmt_num(valor: float, casas: int = 3) -> str:
    if valor is None or not math.isfinite(float(valor)):
        return "NA"

    return f"{float(valor):.{casas}f}"


def avaliar_operacional(
    documento,
    foguete,
    simulacao_base,
) -> Dict:
    """
    Testa 10, 8, 6, 4, 2 e 0 m/s, nessa ordem.

    Em CADA vento, exige simultaneamente:
      - S_saida >= ESTABILIDADE_MINIMA_SAIDA
      - S_max durante TODO o voo <= ESTABILIDADE_MAXIMA

    Para assim que encontra a primeira falha.
    """
    resultados = {}
    maior_estabilidade = -math.inf
    menor_estabilidade_saida = math.inf

    for vento in VENTOS_OPERACIONAIS:
        try:
            r = simular_vento(
                documento,
                foguete,
                simulacao_base,
                vento,
            )
        except Exception as exc:
            return {
                "valido": False,
                "tipo": "ERRO_SIM",
                "vento_falha": vento,
                "motivo": str(exc),
                "resultados": resultados,
                "smax_global": (
                    maior_estabilidade
                    if math.isfinite(maior_estabilidade)
                    else float("nan")
                ),
                "ssaida_min_global": (
                    menor_estabilidade_saida
                    if math.isfinite(menor_estabilidade_saida)
                    else float("nan")
                ),
                "falta_minima": 0.0,
                "excesso_maximo": 0.0,
            }

        resultados[vento] = r

        s_saida = float(r["estabilidade_saida"])
        s_max = float(r["estabilidade_maxima"])

        maior_estabilidade = max(
            maior_estabilidade,
            s_max,
        )
        menor_estabilidade_saida = min(
            menor_estabilidade_saida,
            s_saida,
        )

        falta_minima = max(
            0.0,
            ESTABILIDADE_MINIMA_SAIDA - s_saida,
        )
        excesso_maximo = max(
            0.0,
            s_max - ESTABILIDADE_MAXIMA,
        )

        if falta_minima > 0.0 or excesso_maximo > 0.0:
            motivos = []

            if falta_minima > 0.0:
                motivos.append(
                    f"S_saida={s_saida:.3f} cal < "
                    f"{ESTABILIDADE_MINIMA_SAIDA:.3f} cal"
                )

            if excesso_maximo > 0.0:
                motivos.append(
                    f"S_max_voo={s_max:.3f} cal > "
                    f"{ESTABILIDADE_MAXIMA:.3f} cal"
                )

            return {
                "valido": False,
                "tipo": "INVALIDO",
                "vento_falha": vento,
                "motivo": " | ".join(motivos),
                "resultados": resultados,
                "smax_global": maior_estabilidade,
                "ssaida_min_global": menor_estabilidade_saida,
                "falta_minima": falta_minima,
                "excesso_maximo": excesso_maximo,
            }

    return {
        "valido": True,
        "tipo": "VALIDO",
        "vento_falha": None,
        "motivo": "",
        "resultados": resultados,
        "smax_global": maior_estabilidade,
        "ssaida_min_global": menor_estabilidade_saida,
        "falta_minima": 0.0,
        "excesso_maximo": 0.0,
        "apogeu_0": float(resultados[0.0]["apogeu"]),
    }


def avaliar_seguranca(
    documento,
    foguete,
    simulacao_base,
) -> Dict:
    """
    Testa 15 m/s depois da faixa operacional.

    A regra e' exatamente a mesma dos outros ventos:
      - S_saida >= ESTABILIDADE_MINIMA_SAIDA
      - S_max durante TODO o voo <= ESTABILIDADE_MAXIMA
    """
    vento = VENTO_SEGURANCA

    try:
        r = simular_vento(
            documento,
            foguete,
            simulacao_base,
            vento,
        )
    except Exception as exc:
        return {
            "valido": False,
            "tipo": "ERRO_SIM",
            "motivo": str(exc),
            "resultado": None,
            "falta_minima": 0.0,
            "excesso_maximo": 0.0,
        }

    s_saida = float(r["estabilidade_saida"])
    s_max = float(r["estabilidade_maxima"])

    falta_minima = max(
        0.0,
        ESTABILIDADE_MINIMA_SAIDA - s_saida,
    )

    excesso_maximo = max(
        0.0,
        s_max - ESTABILIDADE_MAXIMA,
    )

    valido = (
        falta_minima <= 0.0
        and excesso_maximo <= 0.0
    )

    motivos = []

    if falta_minima > 0.0:
        motivos.append(
            f"S_saida={s_saida:.3f} cal < "
            f"{ESTABILIDADE_MINIMA_SAIDA:.3f} cal"
        )

    if excesso_maximo > 0.0:
        motivos.append(
            f"S_max_voo={s_max:.3f} cal > "
            f"{ESTABILIDADE_MAXIMA:.3f} cal"
        )

    return {
        "valido": valido,
        "tipo": "VALIDO" if valido else "INVALIDO",
        "motivo": " | ".join(motivos),
        "resultado": r,
        "falta_minima": falta_minima,
        "excesso_maximo": excesso_maximo,
    }


def imprimir_avaliacao(
    numero: int,
    x,
    operacional: Dict,
    seguranca: Optional[Dict],
) -> None:
    (
        raiz, ponta, altura, sweep,
        distancia_boattail_m, comprimento_ogiva_m,
    ) = map(float, x)

    print()
    print(
        f"Avaliacao {numero:4d} | "
        f"raiz={cm(raiz):6.2f} "
        f"ponta={cm(ponta):6.2f} "
        f"altura={cm(altura):6.2f} "
        f"sweep={cm(sweep):6.2f} cm"
    )
    print(
        f"                 | "
        f"dist_boattail={cm(distancia_boattail_m):6.2f} cm "
        f"| ogiva={cm(comprimento_ogiva_m):6.2f} cm"
    )

    # Linha 1: operacional
    if operacional["valido"]:
        print(
            "  OPERACIONAL [10, 8, 6, 4, 2, 0 m/s] | VALIDO"
            f" | Ssaida_min={operacional['ssaida_min_global']:.3f} cal"
            f" | Smax_voo={operacional['smax_global']:.3f} cal"
            f" | apogeu_0={operacional['apogeu_0']:.2f} m"
        )
    else:
        vento = operacional.get("vento_falha")
        tipo = operacional.get("tipo", "INVALIDO")
        motivo = operacional.get("motivo", "")

        print(
            "  OPERACIONAL [10, 8, 6, 4, 2, 0 m/s] | "
            f"{tipo}"
            + (
                f" em {vento:g} m/s"
                if vento is not None
                else ""
            )
            + (
                f" | {motivo}"
                if motivo
                else ""
            )
        )

    # Linha 2: seguranca
    if seguranca is None:
        print(
            "  SEGURANCA   [15 m/s]                | NAO TESTADO"
        )
    elif seguranca["valido"]:
        r = seguranca["resultado"]

        print(
            "  SEGURANCA   [15 m/s]                | VALIDO"
            f" | S_saida={r['estabilidade_saida']:.3f} cal"
            f" | Smax_voo={r['estabilidade_maxima']:.3f} cal"
        )
    else:
        r = seguranca.get("resultado")
        tipo = seguranca.get("tipo", "INVALIDO")

        if r is not None:
            print(
                "  SEGURANCA   [15 m/s]                | "
                f"{tipo}"
                f" | S_saida={r['estabilidade_saida']:.3f} cal"
                f" | Smax_voo={r['estabilidade_maxima']:.3f} cal"
                + (
                    f" | {seguranca['motivo']}"
                    if seguranca.get("motivo")
                    else ""
                )
            )
        else:
            print(
                "  SEGURANCA   [15 m/s]                | "
                f"{tipo}"
                + (
                    f" | {seguranca['motivo']}"
                    if seguranca.get("motivo")
                    else ""
                )
            )



def violacao_regras_do_candidato(
    operacional: Dict,
    seguranca: Optional[Dict],
) -> Optional[float]:
    """
    Retorna a violacao, em calibres, da primeira condicao que reprovou
    o candidato.

    ERRO_SIM e restricoes geometricas nao entram no ranking de
    "menor violacao", pois nao representam uma violacao quantitativa
    das regras de estabilidade.
    """
    if operacional.get("tipo") in ("ERRO_SIM", "INVALIDO_GEOMETRIA"):
        return None

    if not operacional.get("valido", False):
        falta = float(operacional.get("falta_minima", 0.0))
        excesso = float(operacional.get("excesso_maximo", 0.0))
        return falta + excesso

    if seguranca is None:
        return None

    if seguranca.get("tipo") == "ERRO_SIM":
        return None

    if not seguranca.get("valido", False):
        falta = float(seguranca.get("falta_minima", 0.0))
        excesso = float(seguranca.get("excesso_maximo", 0.0))
        return falta + excesso

    return None


def quantidade_ventos_aprovados(
    operacional: Dict,
    seguranca: Optional[Dict],
) -> int:
    """
    Conta quantos ventos foram efetivamente simulados e passaram nas duas regras.

    Serve apenas como desempate do ranking de menor violacao.
    """
    aprovados = 0

    for r in operacional.get("resultados", {}).values():
        s_saida = float(r["estabilidade_saida"])
        s_max = float(r["estabilidade_maxima"])

        if (
            s_saida >= ESTABILIDADE_MINIMA_SAIDA
            and s_max <= ESTABILIDADE_MAXIMA
        ):
            aprovados += 1

    if seguranca is not None:
        r15 = seguranca.get("resultado")

        if r15 is not None:
            s_saida = float(r15["estabilidade_saida"])
            s_max = float(r15["estabilidade_maxima"])

            if (
                s_saida >= ESTABILIDADE_MINIMA_SAIDA
                and s_max <= ESTABILIDADE_MAXIMA
            ):
                aprovados += 1

    return aprovados


def apogeu_0_para_ranking_invalido(
    documento,
    foguete,
    simulacao_base,
    operacional: Dict,
) -> Tuple[float, str]:
    """
    Obtem o apogeu a 0 m/s para comparar candidatos invalidos.

    Se 0 m/s ja tiver sido simulado no fluxo operacional, reaproveita
    o resultado. Caso contrario, executa UMA simulacao extra a 0 m/s.

    Retorna:
        (apogeu_em_metros, origem)

    Se a simulacao de 0 m/s falhar, retorna (nan, mensagem).
    """
    resultados = operacional.get("resultados", {})

    if 0.0 in resultados:
        try:
            return float(resultados[0.0]["apogeu"]), "fluxo_operacional"
        except Exception:
            pass

    try:
        r0 = simular_vento(
            documento,
            foguete,
            simulacao_base,
            0.0,
        )
        return float(r0["apogeu"]), "simulacao_extra_0_m_s"
    except Exception as exc:
        return float("nan"), f"falhou_0_m_s: {exc}"


def motivo_reprovacao_resumido(
    operacional: Dict,
    seguranca: Optional[Dict],
) -> str:
    if not operacional.get("valido", False):
        vento = operacional.get("vento_falha")
        motivo = operacional.get("motivo", "")

        if vento is None:
            return motivo or operacional.get("tipo", "INVALIDO")

        return f"{vento:g} m/s: {motivo}"

    if seguranca is not None and not seguranca.get("valido", False):
        motivo = seguranca.get("motivo", "")
        return f"15 m/s: {motivo}"

    return "motivo nao identificado"


def registro_invalido(
    numero: int,
    x,
    operacional: Dict,
    seguranca: Optional[Dict],
    apogeu_0: float,
    origem_apogeu: str,
) -> Dict:
    (
        raiz, ponta, altura, sweep,
        distancia_boattail_m, comprimento_ogiva_m,
    ) = map(float, x)

    violacao = violacao_regras_do_candidato(
        operacional,
        seguranca,
    )

    return {
        "avaliacao": int(numero),
        "x": tuple(map(float, x)),
        "raiz_cm": cm(raiz),
        "ponta_cm": cm(ponta),
        "altura_cm": cm(altura),
        "sweep_cm": cm(sweep),
        "distancia_boattail_cm": cm(distancia_boattail_m),
        "comprimento_ogiva_cm": cm(comprimento_ogiva_m),
        "area_aleta_cm2": area_aleta_m2(x) * 10000.0,
        "violacao_cal": (
            float(violacao)
            if violacao is not None
            else float("nan")
        ),
        "ventos_aprovados": quantidade_ventos_aprovados(
            operacional,
            seguranca,
        ),
        "apogeu_0_m": float(apogeu_0),
        "origem_apogeu": str(origem_apogeu),
        "motivo": motivo_reprovacao_resumido(
            operacional,
            seguranca,
        ),
        "operacional": operacional,
        "seguranca": seguranca,
    }


def chave_menor_violacao(registro: Dict):
    """
    Menor tupla = melhor.

    1) menor violacao em cal;
    2) em empate, MAIS ventos aprovados;
    3) em novo empate, MAIOR apogeu.
    """
    violacao = float(registro["violacao_cal"])
    aprovados = int(registro["ventos_aprovados"])
    apogeu = float(registro["apogeu_0_m"])

    if not math.isfinite(apogeu):
        apogeu = -math.inf

    return (
        violacao,
        -aprovados,
        -apogeu,
    )


def imprimir_melhor_invalido(
    titulo: str,
    registro: Optional[Dict],
) -> None:
    print()
    print("-" * 82)
    print(titulo)
    print("-" * 82)

    if registro is None:
        print("Nenhum candidato adequado para este ranking.")
        return

    print(f"Avaliacao      : {registro['avaliacao']}")
    print(f"Corda da raiz : {registro['raiz_cm']:.4f} cm")
    print(f"Corda da ponta: {registro['ponta_cm']:.4f} cm")
    print(f"Altura        : {registro['altura_cm']:.4f} cm")
    print(f"Sweep         : {registro['sweep_cm']:.4f} cm")
    print(
        f"Dist. boattail: "
        f"{registro['distancia_boattail_cm']:.4f} cm"
    )
    print(
        f"Comp. ogiva   : "
        f"{registro['comprimento_ogiva_cm']:.4f} cm"
    )
    print(f"Area / aleta  : {registro['area_aleta_cm2']:.3f} cm^2")

    violacao = float(registro["violacao_cal"])

    if math.isfinite(violacao):
        print(f"Violacao      : {violacao:.5f} cal")
    else:
        print("Violacao      : NA")

    print(
        f"Ventos aprov. : {registro['ventos_aprovados']} de 7"
    )

    apogeu = float(registro["apogeu_0_m"])

    if math.isfinite(apogeu):
        print(f"Apogeu 0 m/s  : {apogeu:.2f} m")
    else:
        print("Apogeu 0 m/s  : NA")

    print(f"Motivo        : {registro['motivo']}")


def escrever_melhores_invalidos_csv(
    menor_violacao: Optional[Dict],
    maior_apogeu: Optional[Dict],
) -> None:
    registros = [
        ("MENOR_VIOLACAO", menor_violacao),
        ("MAIOR_APOGEU", maior_apogeu),
    ]

    campos = [
        "criterio",
        "avaliacao",
        "raiz_cm",
        "ponta_cm",
        "altura_cm",
        "sweep_cm",
        "distancia_boattail_cm",
        "comprimento_ogiva_cm",
        "area_aleta_cm2",
        "violacao_cal",
        "ventos_aprovados",
        "apogeu_0_m",
        "origem_apogeu",
        "motivo_reprovacao",
    ]

    with ARQUIVO_MELHORES_INVALIDOS.open(
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as arquivo:
        writer = csv.DictWriter(
            arquivo,
            fieldnames=campos,
            delimiter=";",
        )
        writer.writeheader()

        for criterio, registro in registros:
            if registro is None:
                continue

            writer.writerow({
                "criterio": criterio,
                "avaliacao": registro["avaliacao"],
                "raiz_cm": f"{registro['raiz_cm']:.5f}",
                "ponta_cm": f"{registro['ponta_cm']:.5f}",
                "altura_cm": f"{registro['altura_cm']:.5f}",
                "sweep_cm": f"{registro['sweep_cm']:.5f}",
                "distancia_boattail_cm": (
                    f"{registro['distancia_boattail_cm']:.5f}"
                ),
                "comprimento_ogiva_cm": (
                    f"{registro['comprimento_ogiva_cm']:.5f}"
                ),
                "area_aleta_cm2": (
                    f"{registro['area_aleta_cm2']:.5f}"
                ),
                "violacao_cal": fmt_num(
                    registro["violacao_cal"],
                    6,
                ),
                "ventos_aprovados": registro["ventos_aprovados"],
                "apogeu_0_m": fmt_num(
                    registro["apogeu_0_m"],
                    5,
                ),
                "origem_apogeu": registro["origem_apogeu"],
                "motivo_reprovacao": registro["motivo"],
            })


# ============================================================================
# 6. CSV / SALVAMENTO
# ============================================================================

CAMPOS_HISTORICO = [
    "avaliacao",
    "raiz_cm",
    "ponta_cm",
    "altura_cm",
    "sweep_cm",
    "distancia_boattail_cm",
    "comprimento_ogiva_cm",
    "area_aleta_cm2",
    "operacional_status",
    "operacional_vento_falha",
    "operacional_motivo",
    "seguranca_status",
    "seguranca_motivo",
    "apogeu_0_m",
    "s_saida_min_operacional_cal",
    "s_saida_15_cal",
    "s_max_voo_15_cal",
    "s_max_voo_global_cal",
    "objetivo",
]


def escrever_linha_historico(
    hist,
    numero: int,
    x,
    operacional: Dict,
    seguranca: Optional[Dict],
    objetivo: float,
) -> None:
    (
        raiz, ponta, altura, sweep,
        distancia_boattail_m, comprimento_ogiva_m,
    ) = map(float, x)

    apogeu_0 = float("nan")

    if operacional.get("valido"):
        apogeu_0 = float(operacional["apogeu_0"])

    s_saida_min_operacional = float(
        operacional.get("ssaida_min_global", float("nan"))
    )

    s_saida_15 = float("nan")
    s_max_15 = float("nan")
    seg_status = "NAO_TESTADO"
    seg_motivo = ""

    if seguranca is not None:
        seg_status = seguranca.get("tipo", "")
        seg_motivo = seguranca.get("motivo", "")

        r15 = seguranca.get("resultado")

        if r15 is not None:
            s_saida_15 = float(r15["estabilidade_saida"])
            s_max_15 = float(r15["estabilidade_maxima"])

    smax_global = operacional.get(
        "smax_global",
        float("nan"),
    )

    if math.isfinite(s_max_15):
        if math.isfinite(float(smax_global)):
            smax_global = max(
                float(smax_global),
                s_max_15,
            )
        else:
            smax_global = s_max_15

    hist.writerow({
        "avaliacao": numero,
        "raiz_cm": f"{cm(raiz):.5f}",
        "ponta_cm": f"{cm(ponta):.5f}",
        "altura_cm": f"{cm(altura):.5f}",
        "sweep_cm": f"{cm(sweep):.5f}",
        "distancia_boattail_cm": (
            f"{cm(distancia_boattail_m):.5f}"
        ),
        "comprimento_ogiva_cm": (
            f"{cm(comprimento_ogiva_m):.5f}"
        ),
        "area_aleta_cm2": f"{area_aleta_m2(x) * 10000.0:.5f}",
        "operacional_status": operacional.get("tipo", ""),
        "operacional_vento_falha": (
            ""
            if operacional.get("vento_falha") is None
            else f"{operacional['vento_falha']:.3f}"
        ),
        "operacional_motivo": operacional.get("motivo", ""),
        "seguranca_status": seg_status,
        "seguranca_motivo": seg_motivo,
        "apogeu_0_m": fmt_num(apogeu_0, 5),
        "s_saida_min_operacional_cal": fmt_num(
            s_saida_min_operacional, 5
        ),
        "s_saida_15_cal": fmt_num(s_saida_15, 5),
        "s_max_voo_15_cal": fmt_num(s_max_15, 5),
        "s_max_voo_global_cal": fmt_num(float(smax_global), 5),
        "objetivo": f"{float(objetivo):.8f}",
    })


def escrever_resultado_csv(
    resultados: List[Dict[str, float | str]]
) -> None:
    campos = [
        "grupo",
        "vento_m_s",
        "estabilidade_saida_cal",
        "estabilidade_maxima_todo_voo_cal",
        "velocidade_saida_trilho_m_s",
        "apogeu_m",
        "tempo_saida_s",
        "tempo_apogeu_s",
        "avisos_openrocket",
    ]

    with ARQUIVO_RESULTADO.open(
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as arquivo:
        writer = csv.DictWriter(
            arquivo,
            fieldnames=campos,
            delimiter=";",
        )

        writer.writeheader()

        for r in resultados:
            vento = float(r["vento"])

            grupo = (
                "SEGURANCA"
                if abs(vento - VENTO_SEGURANCA) < 1e-9
                else "OPERACIONAL"
            )

            writer.writerow({
                "grupo": grupo,
                "vento_m_s": f"{vento:.3f}",
                "estabilidade_saida_cal": (
                    f"{float(r['estabilidade_saida']):.5f}"
                ),
                "estabilidade_maxima_todo_voo_cal": (
                    f"{float(r['estabilidade_maxima']):.5f}"
                ),
                "velocidade_saida_trilho_m_s": (
                    f"{float(r['velocidade_saida']):.5f}"
                ),
                "apogeu_m": f"{float(r['apogeu']):.5f}",
                "tempo_saida_s": f"{float(r['tempo_saida']):.5f}",
                "tempo_apogeu_s": f"{float(r['tempo_apogeu']):.5f}",
                "avisos_openrocket": str(r.get("avisos", "")),
            })


def salvar_ork(documento) -> None:
    GeneralRocketSaver = JClass(
        "info.openrocket.core.file.GeneralRocketSaver"
    )
    JavaFile = JClass("java.io.File")

    saver = GeneralRocketSaver()

    saver.save(
        JavaFile(str(ARQUIVO_ORK_OTIMIZADO)),
        documento,
    )


# ============================================================================
# 7. VALIDACAO COMPLETA DE UMA GEOMETRIA
# ============================================================================

def verificar_geometria_candidato(aleta, boattail, x) -> Tuple[List[str], float]:
    """Restricoes geometricas com posicoes reais (apos aplicar_geometria).

    A raiz deve caber inteiramente no seu tubo pai e terminar antes do
    inicio do boattail. Verifica tambem a distancia realmente produzida.
    """
    raiz, ponta, altura, sweep, dist, comprimento_ogiva = map(float, x)
    motivos = []
    excesso = 0.0
    tol = 1e-6
    if EXIGIR_PONTA_MENOR_IGUAL_RAIZ and ponta > raiz + tol:
        motivos.append("corda da ponta maior que a da raiz")
        excesso += ponta-raiz
    pai = aleta.getParent()
    ini_pai = posicao_absoluta_frente(pai)
    fim_pai = ini_pai + float(pai.getLength())
    ini_aleta = posicao_absoluta_frente(aleta)
    fim_raiz = ini_aleta + float(aleta.getRootChord())
    ini_boattail = posicao_absoluta_frente(boattail)
    if ini_aleta < ini_pai - tol:
        motivos.append("inicio da aleta ultrapassa a frente do tubo pai")
        excesso += ini_pai - ini_aleta
    if fim_raiz > fim_pai + tol:
        motivos.append("bordo de fuga da raiz ultrapassa o final do tubo pai")
        excesso += fim_raiz - fim_pai
    if fim_raiz > ini_boattail + tol:
        motivos.append("raiz da aleta invade o inicio do boattail")
        excesso += fim_raiz - ini_boattail
    if dist < -tol:
        motivos.append("distancia ao boattail negativa")
        excesso += -dist
    dist_real = ini_boattail - fim_raiz
    if abs(dist_real - dist) > tol:
        motivos.append(f"distancia ao boattail aplicada diferente da solicitada: {dist_real:.6f} m")
        excesso += abs(dist_real - dist)
    return motivos, float(excesso)


def validar_geometria_completa(
    documento,
    foguete,
    simulacao_base,
    x,
    imprimir: bool = True,
):
    aplicar_geometria(
        foguete,
        encontrar_aleta(foguete),
        encontrar_ogiva(foguete),
        encontrar_boattail(foguete),
        x,
    )

    motivos_geometria, _ = verificar_geometria_candidato(
        encontrar_aleta(foguete), encontrar_boattail(foguete), x
    )
    if motivos_geometria:
        operacional = {
            "valido": False,
            "tipo": "INVALIDO_GEOMETRIA",
            "vento_falha": None,
            "motivo": " | ".join(motivos_geometria),
            "resultados": {},
        }
        if imprimir:
            imprimir_avaliacao(0, x, operacional, None)
        return False, operacional, None, []

    operacional = avaliar_operacional(
        documento,
        foguete,
        simulacao_base,
    )

    seguranca = None

    if operacional["valido"]:
        seguranca = avaliar_seguranca(
            documento,
            foguete,
            simulacao_base,
        )

    if imprimir:
        imprimir_avaliacao(
            0,
            x,
            operacional,
            seguranca,
        )

    valido = (
        operacional["valido"]
        and seguranca is not None
        and seguranca["valido"]
    )

    resultados = []

    if operacional.get("resultados"):
        for vento in VENTOS_OPERACIONAIS:
            if vento in operacional["resultados"]:
                resultados.append(
                    operacional["resultados"][vento]
                )

    if (
        seguranca is not None
        and seguranca.get("resultado") is not None
    ):
        resultados.append(
            seguranca["resultado"]
        )

    return valido, operacional, seguranca, resultados


# ============================================================================
# 8. PROGRAMA PRINCIPAL
# ============================================================================

def main() -> None:
    print("=" * 82)
    print("OTIMIZADOR DE GEOMETRIA - OPENROCKET 24.12 - V7 6VAR")
    print("=" * 82)
    print(f"ORK   : {ARQUIVO_ORK.name}")
    print(f"Motor : {ARQUIVO_MOTOR.name}")
    print()
    print(
        "OPERACIONAL : 10, 8, 6, 4, 2 e 0 m/s"
        f" | S_saida >= {ESTABILIDADE_MINIMA_SAIDA:.2f} cal"
        f" | S_max_voo <= {ESTABILIDADE_MAXIMA:.2f} cal"
    )
    print(
        f"SEGURANCA   : 15 m/s"
        f" | S_saida >= {ESTABILIDADE_MINIMA_SAIDA:.2f} cal"
        f" | S_max_voo <= {ESTABILIDADE_MAXIMA:.2f} cal"
    )
    print(
        "OBJETIVO    : entre candidatos totalmente validos, "
        "maximizar apogeu em 0 m/s"
    )
    print()

    modo_massa = str(MODO_MASSA_ALETAS).strip().lower()
    modos_validos = {
        "arquivo_ork",
        "material_openrocket",
        "massa_areal",
        "massa_total_fixa",
    }

    if modo_massa not in modos_validos:
        raise RuntimeError(
            f"MODO_MASSA_ALETAS invalido: {MODO_MASSA_ALETAS!r}"
        )

    if modo_massa == "massa_areal" and MASSA_AREAL_ALETAS_KG_M2 is None:
        raise RuntimeError(
            "Preencha MASSA_AREAL_ALETAS_KG_M2 antes de usar "
            "MODO_MASSA_ALETAS='massa_areal'."
        )

    if (
        modo_massa == "massa_total_fixa"
        and MASSA_TOTAL_FIXA_ALETAS_KG is None
    ):
        raise RuntimeError(
            "Preencha MASSA_TOTAL_FIXA_ALETAS_KG antes de usar "
            "MODO_MASSA_ALETAS='massa_total_fixa'."
        )

    # Saidas antigas nao devem ser confundidas com resultados desta execucao.
    for arquivo_antigo in (
        ARQUIVO_ORK_OTIMIZADO, ARQUIVO_RESULTADO, ARQUIVO_MELHORES_INVALIDOS,
    ):
        if arquivo_antigo.exists():
            arquivo_antigo.unlink()

    iniciar_openrocket()

    documento = carregar_documento()
    foguete = documento.getRocket()

    if int(documento.getSimulationCount()) <= INDICE_SIMULACAO_BASE:
        raise RuntimeError(
            f"O .ork nao possui simulacao no indice "
            f"{INDICE_SIMULACAO_BASE}."
        )

    simulacao_base = documento.getSimulation(
        INDICE_SIMULACAO_BASE
    )

    configuracao = simulacao_base.getFlightConfigurationId()

    if not foguete.hasMotors(configuracao):
        raise RuntimeError(
            "O motor da configuracao da simulacao-base nao foi "
            "associado ao .eng carregado."
        )

    print(
        f"Simulacao-base: indice {INDICE_SIMULACAO_BASE}"
        f" | nome='{simulacao_base.getName()}'"
    )

    aleta = encontrar_aleta(foguete)
    ogiva = encontrar_ogiva(foguete)
    boattail = encontrar_boattail(foguete)
    inicial = geometria_atual(aleta, ogiva, boattail)

    print(f"Aleta otimizada: '{aleta.getName()}'")
    print(f"Ogiva otimizada: '{ogiva.getName()}'")
    print(f"Boattail ref.  : '{boattail.getName()}'")
    print(
        "Geometria atual: "
        f"raiz={cm(inicial[0]):.3f} cm | "
        f"ponta={cm(inicial[1]):.3f} cm | "
        f"altura={cm(inicial[2]):.3f} cm | "
        f"sweep={cm(inicial[3]):.3f} cm"
    )
    print(
        f"                 dist_boattail={cm(inicial[4]):.3f} cm | "
        f"comprimento_ogiva={cm(inicial[5]):.3f} cm"
    )
    print(
        f"Massa atual do FinSet no OpenRocket: "
        f"{massa_aletas_atual_kg(aleta):.4f} kg"
    )
    print(f"Modelo de massa escolhido: {MODO_MASSA_ALETAS}")
    if MODO_MASSA_ALETAS == "arquivo_ork" and bool(aleta.isMassOverridden()):
        print("AVISO: a massa das aletas tem override no .ork: "
              "a massa pode ficar constante mesmo mudando a area.")

    # Aviso se a geometria atual estiver fora dos limites.
    ordem_nomes = (
        "corda_raiz",
        "corda_ponta",
        "altura",
        "sweep",
        "distancia_boattail",
        "comprimento_ogiva",
    )

    fora = []

    for nome, valor_m in zip(ordem_nomes, inicial):
        minimo, maximo = LIMITES_CM[nome]
        valor_cm = cm(valor_m)

        if valor_cm < minimo or valor_cm > maximo:
            fora.append(
                f"{nome}={valor_cm:.3f} cm "
                f"(busca: {minimo:.3f} a {maximo:.3f})"
            )

    if fora:
        print()
        print(
            "AVISO: a geometria atual esta fora de alguns limites "
            "da busca:"
        )

        for item in fora:
            print("  - " + item)

        print(
            "Isso nao impede o teste do foguete atual; "
            "apenas limita onde o otimizador podera procurar."
        )

    inicial_dentro_limites = not fora

    # Aplica explicitamente a configuracao atual usando o modelo de massa
    # escolhido. Isso garante que o teste inicial e a busca usem a mesma
    # regra de massa.
    aplicar_geometria(
        foguete,
        aleta,
        ogiva,
        boattail,
        inicial,
    )

    # ----------------------------------------------------------------------
    # Teste inicial
    # ----------------------------------------------------------------------

    print()
    print("=" * 82)
    print("TESTE DA GEOMETRIA ATUAL")
    print("=" * 82)

    operacional_atual = avaliar_operacional(
        documento,
        foguete,
        simulacao_base,
    )

    seguranca_atual = None

    if operacional_atual["valido"]:
        seguranca_atual = avaliar_seguranca(
            documento,
            foguete,
            simulacao_base,
        )

    imprimir_avaliacao(
        0,
        inicial,
        operacional_atual,
        seguranca_atual,
    )

    # A geometria original TAMBEM concorre como solucao, se for viavel.
    motivos_inicial, _ = verificar_geometria_candidato(aleta, boattail, inicial)
    inicial_valido = (
        inicial_dentro_limites and not motivos_inicial
        and operacional_atual.get("valido", False)
        and seguranca_atual is not None and seguranca_atual.get("valido", False)
    )
    if motivos_inicial:
        print("Geometria inicial fora do tubo/implantacao: " + " | ".join(motivos_inicial))
    if inicial_valido:
        print("A geometria inicial esta VALIDA e participara da comparacao de apogeu.")

    # Volta para a geometria original antes de iniciar a busca.
    aplicar_geometria(
        foguete,
        aleta,
        ogiva,
        boattail,
        inicial,
    )

    # ----------------------------------------------------------------------
    # Otimizacao
    # ----------------------------------------------------------------------

    print()
    print("=" * 82)
    print("INICIANDO DIFFERENTIAL EVOLUTION")
    print("=" * 82)
    print(
        f"MAXITER={MAXITER} | POPSIZE={POPSIZE} "
        f"| 6 variaveis"
    )
    print()

    numero_avaliacao = 0

    # Cache: geometria -> objetivo
    cache: Dict[Tuple[float, ...], float] = {}

    # Melhor candidato que REALMENTE passou em tudo.
    melhor_valido_x: Optional[Tuple[float, ...]] = (
        tuple(map(float, inicial)) if inicial_valido else None
    )
    melhor_valido_apogeu = (
        float(operacional_atual["apogeu_0"]) if inicial_valido else -math.inf
    )

    # Dois rankings independentes de candidatos INVALIDOS.
    melhor_invalido_violacao: Optional[Dict] = None
    melhor_invalido_apogeu: Optional[Dict] = None

    with ARQUIVO_HISTORICO.open(
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as arquivo_hist:

        hist = csv.DictWriter(
            arquivo_hist,
            fieldnames=CAMPOS_HISTORICO,
            delimiter=";",
        )

        hist.writeheader()
        # Linha 0 registra o teste inicial, distinto das avaliacoes da busca.
        escrever_linha_historico(
            hist, 0, inicial, operacional_atual, seguranca_atual,
            -float(operacional_atual["apogeu_0"]) if inicial_valido else 50_000_000.0,
        )
        arquivo_hist.flush()

        def objetivo(x) -> float:
            nonlocal numero_avaliacao
            nonlocal melhor_valido_x
            nonlocal melhor_valido_apogeu
            nonlocal melhor_invalido_violacao
            nonlocal melhor_invalido_apogeu

            x = tuple(map(float, x))

            # Evita recalcular o mesmo ponto por arredondamento.
            chave = tuple(
                round(valor, 8)
                for valor in x
            )

            if chave in cache:
                return cache[chave]

            numero_avaliacao += 1

            (
                raiz, ponta, altura, sweep,
                distancia_boattail_m, comprimento_ogiva_m,
            ) = x

            # ----------------------------------------------------------
            # Aplica e verifica a geometria REAL antes de simular ventos.
            # Nao compara so comprimentos: inspeciona posicoes absolutas.
            # ----------------------------------------------------------
            try:
                aplicar_geometria(foguete, aleta, ogiva, boattail, x)
                motivos_geometria, excesso_geometrico = verificar_geometria_candidato(
                    aleta, boattail, x
                )
            except Exception as exc:
                motivos_geometria = [f"erro ao aplicar/verificar geometria: {exc}"]
                excesso_geometrico = 1.0

            if motivos_geometria:
                operacional = {
                    "valido": False, "tipo": "INVALIDO_GEOMETRIA",
                    "vento_falha": None,
                    "motivo": " | ".join(motivos_geometria),
                    "resultados": {}, "smax_global": float('nan'),
                }
                seguranca = None
                valor_objetivo = 20_000_000.0 + excesso_geometrico * 1_000_000.0
                if IMPRIMIR_TODAS_AVALIACOES:
                    imprimir_avaliacao(numero_avaliacao, x, operacional, seguranca)
                escrever_linha_historico(
                    hist, numero_avaliacao, x, operacional, seguranca, valor_objetivo
                )
                arquivo_hist.flush()
                cache[chave] = valor_objetivo
                return valor_objetivo

            # ----------------------------------------------------------
            # ETAPA 1 - OPERACIONAL
            # ----------------------------------------------------------

            operacional = avaliar_operacional(
                documento,
                foguete,
                simulacao_base,
            )

            seguranca = None

            if not operacional["valido"]:
                if operacional["tipo"] == "ERRO_SIM":
                    valor_objetivo = 50_000_000.0
                else:
                    violacao = float(operacional.get("falta_minima", 0.0)) + float(
                        operacional.get("excesso_maximo", 0.0))
                    aprovados = quantidade_ventos_aprovados(operacional, None)
                    # Penaliza quantidade de condicoes ainda nao aprovadas
                    # e a violacao na primeira condicao reprovada.
                    valor_objetivo = (1_000_000.0 + (7-aprovados)*100_000.0
                                      + violacao*30_000.0)
            else:
                seguranca = avaliar_seguranca(documento, foguete, simulacao_base)
                if seguranca["valido"]:
                    apogeu_0 = float(operacional["apogeu_0"])
                    valor_objetivo = -apogeu_0
                    if apogeu_0 > melhor_valido_apogeu:
                        melhor_valido_apogeu = apogeu_0
                        melhor_valido_x = x
                elif seguranca["tipo"] == "ERRO_SIM":
                    valor_objetivo = 50_000_000.0
                else:
                    violacao = float(seguranca.get("falta_minima", 0.0)) + float(
                        seguranca.get("excesso_maximo", 0.0))
                    valor_objetivo = 1_100_000.0 + violacao*30_000.0

            # ----------------------------------------------------------
            # Rankings dos INVALIDOS
            # ----------------------------------------------------------

            candidato_totalmente_valido = (
                operacional.get("valido", False)
                and seguranca is not None
                and seguranca.get("valido", False)
            )

            candidato_com_erro_sim = (
                operacional.get("tipo") == "ERRO_SIM"
                or (
                    seguranca is not None
                    and seguranca.get("tipo") == "ERRO_SIM"
                )
            )

            candidato_geometria_rejeitada = (
                operacional.get("tipo") == "INVALIDO_GEOMETRIA"
            )

            if (
                not candidato_totalmente_valido
                and not candidato_com_erro_sim
                and not candidato_geometria_rejeitada
            ):
                # Para o ranking de maior apogeu precisamos conhecer
                # o apogeu a 0 m/s mesmo quando a sequencia operacional
                # foi interrompida antes de chegar a 0 m/s.
                apogeu_ranking, origem_apogeu = (
                    apogeu_0_para_ranking_invalido(
                        documento,
                        foguete,
                        simulacao_base,
                        operacional,
                    )
                )

                reg_inv = registro_invalido(
                    numero_avaliacao,
                    x,
                    operacional,
                    seguranca,
                    apogeu_ranking,
                    origem_apogeu,
                )

                violacao = float(reg_inv["violacao_cal"])

                if math.isfinite(violacao):
                    if (
                        melhor_invalido_violacao is None
                        or chave_menor_violacao(reg_inv)
                        < chave_menor_violacao(
                            melhor_invalido_violacao
                        )
                    ):
                        melhor_invalido_violacao = reg_inv

                apogeu_inv = float(reg_inv["apogeu_0_m"])

                if math.isfinite(apogeu_inv):
                    if (
                        melhor_invalido_apogeu is None
                        or apogeu_inv
                        > float(
                            melhor_invalido_apogeu["apogeu_0_m"]
                        )
                    ):
                        melhor_invalido_apogeu = reg_inv

            # ----------------------------------------------------------
            # Terminal / historico
            # ----------------------------------------------------------

            if IMPRIMIR_TODAS_AVALIACOES:
                imprimir_avaliacao(
                    numero_avaliacao,
                    x,
                    operacional,
                    seguranca,
                )

            escrever_linha_historico(
                hist,
                numero_avaliacao,
                x,
                operacional,
                seguranca,
                valor_objetivo,
            )

            arquivo_hist.flush()

            cache[chave] = valor_objetivo

            return valor_objetivo

        resultado_otimizador = differential_evolution(
            objetivo,
            bounds=limites_em_metros(),
            strategy="best1bin",
            maxiter=MAXITER,
            popsize=POPSIZE,
            tol=TOL,
            mutation=(0.5, 1.0),
            recombination=0.7,
            seed=SEMENTE_OTIMIZADOR,
            polish=False,
            workers=1,
            updating="immediate",
            disp=False,
        )

    # ----------------------------------------------------------------------
    # Resultado
    # ----------------------------------------------------------------------

    print()
    print("=" * 82)
    print("RESULTADO DA BUSCA")
    print("=" * 82)
    print(
        f"Avaliacoes realmente simuladas: {numero_avaliacao}"
    )
    print(
        f"Resultado bruto do Differential Evolution: "
        f"{resultado_otimizador.fun:.6f}"
    )

    # Salva e apresenta os dois melhores invalidos encontrados.
    if (
        melhor_invalido_violacao is not None
        or melhor_invalido_apogeu is not None
    ):
        escrever_melhores_invalidos_csv(
            melhor_invalido_violacao,
            melhor_invalido_apogeu,
        )

        print()
        print("=" * 82)
        print("REFERENCIAS ENTRE OS CANDIDATOS INVALIDOS")
        print("=" * 82)

        imprimir_melhor_invalido(
            "MELHOR INVALIDO - MENOR VIOLACAO DAS REGRAS",
            melhor_invalido_violacao,
        )

        imprimir_melhor_invalido(
            "MELHOR INVALIDO - MAIOR APOGEU EM 0 m/s",
            melhor_invalido_apogeu,
        )

        print()
        print(
            f"Resumo dos invalidos: {ARQUIVO_MELHORES_INVALIDOS.name}"
        )

    if melhor_valido_x is None:
        print()
        print(
            "NENHUMA GEOMETRIA TOTALMENTE VALIDA FOI ENCONTRADA "
            "COM OS LIMITES/ITERACOES ATUAIS."
        )
        print(
            "Nenhum foguete_otimizado_6var.ork foi salvo para evitar "
            "apresentar um candidato invalido como solucao."
        )
        print(
            "Use as duas referencias acima para decidir entre:"
        )
        print(
            "  - candidato mais proximo das regras;"
        )
        print(
            "  - candidato invalido de maior desempenho em apogeu."
        )
        print(
            f"Historico completo : {ARQUIVO_HISTORICO.name}"
        )
        if (
            melhor_invalido_violacao is not None
            or melhor_invalido_apogeu is not None
        ):
            print(
                f"Melhores invalidos : {ARQUIVO_MELHORES_INVALIDOS.name}"
            )
        return

    melhor = melhor_valido_x

    aplicar_geometria(
        foguete,
        aleta,
        ogiva,
        boattail,
        melhor,
    )

    print()
    print("MELHOR CANDIDATO VALIDO ENCONTRADO")
    print(
        f"Corda da raiz : {cm(melhor[0]):.4f} cm"
    )
    print(
        f"Corda da ponta: {cm(melhor[1]):.4f} cm"
    )
    print(
        f"Altura        : {cm(melhor[2]):.4f} cm"
    )
    print(
        f"Sweep         : {cm(melhor[3]):.4f} cm"
    )
    print(
        f"Dist. boattail: {cm(melhor[4]):.4f} cm"
    )
    print(
        f"Comp. ogiva   : {cm(melhor[5]):.4f} cm"
    )
    print(
        f"Area / aleta  : "
        f"{area_aleta_m2(melhor) * 10000.0:.3f} cm^2"
    )
    print(
        f"Massa FinSet  : {massa_aletas_atual_kg(aleta):.5f} kg "
        f"({MODO_MASSA_ALETAS})"
    )

    # A verificacao geometrica tambem e repetida antes do salvamento.
    motivos_finais, _ = verificar_geometria_candidato(aleta, boattail, melhor)
    if motivos_finais:
        print("ERRO NA VALIDACAO GEOMETRICA FINAL: " + " | ".join(motivos_finais))
        print("Nenhum .ork sera salvo.")
        return

    # ----------------------------------------------------------------------
    # Validacao final, do zero, antes de salvar.
    # ----------------------------------------------------------------------

    print()
    print("=" * 82)
    print("VALIDACAO FINAL")
    print("=" * 82)

    operacional_final = avaliar_operacional(
        documento,
        foguete,
        simulacao_base,
    )

    seguranca_final = None

    if operacional_final["valido"]:
        seguranca_final = avaliar_seguranca(
            documento,
            foguete,
            simulacao_base,
        )

    imprimir_avaliacao(
        numero_avaliacao + 1,
        melhor,
        operacional_final,
        seguranca_final,
    )

    tudo_ok = (
        operacional_final["valido"]
        and seguranca_final is not None
        and seguranca_final["valido"]
    )

    if not tudo_ok:
        print()
        print(
            "ATENCAO: o candidato perdeu a validacao final."
        )
        print(
            "O arquivo .ork otimizado NAO sera salvo."
        )
        return

    # Junta resultados na ordem desejada:
    # 10, 8, 6, 4, 2, 0 e depois 15.
    resultados_finais = []

    for vento in VENTOS_OPERACIONAIS:
        resultados_finais.append(
            operacional_final["resultados"][vento]
        )

    resultados_finais.append(
        seguranca_final["resultado"]
    )

    escrever_resultado_csv(
        resultados_finais
    )

    salvar_ork(
        documento
    )

    print()
    print("=" * 82)
    print("APROVADO")
    print("=" * 82)
    print(
        f"Apogeu em 0 m/s : "
        f"{operacional_final['apogeu_0']:.2f} m"
    )
    print(
        f"S_saida em 15 m/s: "
        f"{seguranca_final['resultado']['estabilidade_saida']:.3f} cal"
    )
    print(
        f"S_max no voo a 15 m/s: "
        f"{seguranca_final['resultado']['estabilidade_maxima']:.3f} cal"
    )
    print()
    print(
        f"ORK otimizado       : {ARQUIVO_ORK_OTIMIZADO.name}"
    )
    print(
        f"Resultado detalhado : {ARQUIVO_RESULTADO.name}"
    )
    print(
        f"Historico da busca  : {ARQUIVO_HISTORICO.name}"
    )
    if (
        melhor_invalido_violacao is not None
        or melhor_invalido_apogeu is not None
    ):
        print(
            f"Referencias invalidas: {ARQUIVO_MELHORES_INVALIDOS.name}"
        )


if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        print()
        print("Execucao interrompida pelo usuario.")
        raise SystemExit(130)

    except Exception as exc:
        print()
        print("=" * 82)
        print("ERRO")
        print("=" * 82)
        print(exc)
        print()
        print("Detalhes tecnicos:")
        traceback.print_exc()
        raise SystemExit(1)
