/* 
============== AANG - Alimentador Automático de Ração para Gatos ==============
Versão com interface serial e interface web local
Componentes principais:
ESP32 + HX711 + TMC2209 + DS3231 + VL53L0X
*/

// ------------------- Bibliotecas -------------------
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <HX711_ADC.h>
#include <AccelStepper.h>
#include <RTClib.h>
#include <VL53L0X.h>
#include <math.h>

// ------------------- Célula de carga/HX711 -------------------
const int pinoDT  = 18;
const int pinoSCK = 19;

const float fatorInicialCalibracao = 1939.077759f; // obtido por ensaio
HX711_ADC balanca(pinoDT, pinoSCK);

float fatorCalibracao = fatorInicialCalibracao;
bool  calibrado       = true;

float ultimoPesoG        = 0.0f;
bool  novoPesoDisponivel = false;

// ------------------- Motor (TMC2209 em STEP/DIR) -------------------
const int pinoDIR  = 27;
const int pinoSTEP = 26;
const int pinoEN   = 32;

const int passosBasePorVolta  = 200;
const int microPasso          = 16;
const int passosEfetivosVolta = passosBasePorVolta * microPasso;

const float rpmControle = 10.0f;
float passosPorSegundoControle =
    rpmControle * (float)passosEfetivosVolta / 60.0f;

bool driverHabilitado    = false;
bool movimentoMotorAtivo = false;
long alvoPassosMovimento = 0;

AccelStepper motor(AccelStepper::DRIVER, pinoSTEP, pinoDIR);

// ------------------- LEDs -------------------
const int pinoLEDStatusMotor = 13;
const int pinoLEDNivelBaixo  = 4;

// ------------------- Botão de interações manuais -------------------
const int   pinoBotaoDose   = 25;
const float massaDoseBotaoG = 10.0f;

// ------------------- Nível de ração (VL53L0X) -------------------
VL53L0X sensorToF;
bool     sensorToFOk             = false;
bool     nivelReservatorioOk     = true;
uint16_t ultimaDistanciaToF      = 0;   
bool     ultimaLeituraToFValida  = false; 

// distância em mm acima da qual o nível é considerado baixo
const uint16_t limiteDistanciaNivelBaixoMm = 50;

// ------------------- RTC DS3231 -------------------
RTC_DS3231 rtc;
bool rtcOk = false;

// ------------------- Parâmetros de controle -------------------
const float massaPorPassoBaseG  = 0.0307f;
const float massaPorPassoMotorG = massaPorPassoBaseG / (float)microPasso;

const float pRef                 = 0.80f;
const float erroMaxG             = 2.0f;
const float ganhoIncremental     = 0.5f;
const long  passosMaxIncremento  = 800;
const uint32_t tempoAcomodacaoMs = 2000;

const float massaRefPadraoG = 20.0f;
float massaRefG = massaRefPadraoG;

float massaAtualG = 0.0f;
float erroAtualG  = 0.0f;

const uint8_t amostrasMedicaoNecessarias = 10;
uint8_t  contagemAmostrasMedicao         = 0;
double   somaAmostrasMedicao             = 0.0;

bool taraSolicitada = false;

// ------------------- Máquina de estados da refeição -------------------
enum EstadoRefeicao {
  EST_ESPERA_COMANDO,
  EST_INICIALIZACAO,
  EST_PRE_DOSAGEM,
  EST_ACOMODACAO,
  EST_MEDICAO,
  EST_AVALIACAO_ERRO,
  EST_AJUSTE_INCREMENTAL,
  EST_TERMINO_REFEICAO
};

EstadoRefeicao estadoAtual    = EST_ESPERA_COMANDO;
EstadoRefeicao estadoAnterior = EST_ESPERA_COMANDO;

: origem da refeição (manual / botão / agendada)
enum OrigemRefeicao {
  ORIGEM_REFEICAO_DESCONHECIDA = 0,
  ORIGEM_REFEICAO_SERIAL       = 1,
  ORIGEM_REFEICAO_BOTAO        = 2,
  ORIGEM_REFEICAO_AGENDADA     = 3
};

OrigemRefeicao origemRefeicaoAtual       = ORIGEM_REFEICAO_DESCONHECIDA; 
int8_t         slotAgendadoRefeicaoAtual = -1;                            

bool     refeicaoEmProgresso     = false;
uint32_t tempoInicioAcomodacaoMs = 0;

// ------------------- Agendamento de refeições -------------------
struct RefeicaoAgendada {
  bool    habilitada;
  uint8_t hora;
  uint8_t minuto;
  float   massaG;
};

const int maxRefeicoes = 5;
RefeicaoAgendada refeicoes[maxRefeicoes];

int ultimaHoraChecada   = -1;
int ultimoMinutoChecado = -1;

// ------------------- Histórico de refeições (para página /historico) -------------------
struct RegistroHistorico {
  uint16_t ano;
  uint8_t  mes;
  uint8_t  dia;
  uint8_t  hora;
  uint8_t  minuto;
  float    massaRefG;
  float    massaFinalG;
  float    erroFinalG;
  uint8_t  origem;        // usa valores de OrigemRefeicao
  int8_t   slotAgendado;  // -1 se não veio de slot agendado
};

const int maxRegistrosHistorico = 30;  // quantidade máxima de registros armazenados
RegistroHistorico historicoRefeicoes[maxRegistrosHistorico];
int indiceProximoHistorico       = 0;
int quantidadeRegistrosHistorico = 0;

// ------------------- Wi-Fi / Servidor Web -------------------
const char* ssidAP  = "AANG";
const char* senhaAP = "aangatinho";

WebServer servidorWeb(80);

// ------------------- Protótipos de módulos -------------------
bool   VerificarNivelReservatorioOk();
void   ImprimirHoraRTC();
String ObterTextoStatusNivelRacao();       
String ObterClasseCSSStatusNivelRacao();   
void   InicializarHistorico();             
void   RegistrarHistoricoRefeicao();       
String ObterTextoOrigemRefeicao(uint8_t origem, int8_t slotAgendado); 
String GerarPaginaHistoricoHTML();         

// ------------------- LEDs -------------------
void ConfigurarLEDs() {
  pinMode(pinoLEDStatusMotor, OUTPUT);
  digitalWrite(pinoLEDStatusMotor, LOW);

  pinMode(pinoLEDNivelBaixo, OUTPUT);
  digitalWrite(pinoLEDNivelBaixo, LOW);
}

void AtualizarLEDStatusMotor() {
  static uint32_t tUltimaTroca = 0;
  const uint32_t periodoPiscaMs = 150;

  if (!driverHabilitado) {
    digitalWrite(pinoLEDStatusMotor, LOW);
    return;
  }

  if (!movimentoMotorAtivo) {
    digitalWrite(pinoLEDStatusMotor, HIGH);
    return;
  }

  uint32_t agora = millis();
  if (agora - tUltimaTroca >= periodoPiscaMs) {
    tUltimaTroca = agora;
    digitalWrite(pinoLEDStatusMotor, !digitalRead(pinoLEDStatusMotor));
  }
}

// ------------------- Motor -------------------
void AplicarEnableMotor(bool habilitar) {
  driverHabilitado = habilitar;
  digitalWrite(pinoEN, habilitar ? LOW : HIGH);
}

void ConfigurarMotor() {
  pinMode(pinoEN, OUTPUT);
  pinMode(pinoDIR, OUTPUT);

  digitalWrite(pinoDIR, HIGH);
  AplicarEnableMotor(false);

  motor.setMaxSpeed(2000.0f);
  motor.setSpeed(0.0f);
}

void IniciarMovimentoPassos(long passos) {
  if (!driverHabilitado || passos <= 0) return;

  alvoPassosMovimento = motor.currentPosition() + passos;
  motor.setSpeed(passosPorSegundoControle);
  movimentoMotorAtivo = true;
}

void AtualizarMotor() {
  if (movimentoMotorAtivo) {
    motor.runSpeed();

    if (motor.currentPosition() >= alvoPassosMovimento) {
      movimentoMotorAtivo = false;
      motor.setSpeed(0.0f);
    }
  }
}

long ConverterMassaParaPassos(float massaG) {
  if (massaG <= 0.0f) return 0;
  return (long)(massaG / massaPorPassoMotorG + 0.5f);
}

// ------------------- HX711 -------------------
void ConfigurarHX711() {
  balanca.begin();
  balanca.setSamplesInUse(10);
  balanca.setCalFactor(fatorCalibracao);

  const uint16_t tempoEstabilizacaoMs = 2000;
  const bool fazerTaraInicial = true;
  balanca.start(tempoEstabilizacaoMs, fazerTaraInicial);

  if (balanca.getTareTimeoutFlag()) {
    Serial.println("ERRO: tempo excedido na tara inicial. Verifique HX711.");
  } else {
    Serial.println("Tara inicial concluida.");
  }

  Serial.print("Fator de calibracao: ");
  Serial.println(fatorCalibracao, 6);
}

void RealizarTara() {
  balanca.tareNoDelay();
  taraSolicitada = true;
  Serial.println("Tara solicitada...");
}

void VerificarTaraConcluida() {
  if (taraSolicitada && balanca.getTareStatus()) {
    taraSolicitada = false;
    Serial.println("Tara finalizada.");
  }
}

// ------------------- VL53L0X -------------------
void ConfigurarToF() {
  if (!sensorToF.init()) {
    Serial.println("VL53L0X nao encontrado. Nivel nao sera verificado.");
    sensorToFOk            = false;
    ultimaLeituraToFValida = false;
  } else {
    sensorToFOk = true;
    sensorToF.setTimeout(100);
    Serial.println("VL53L0X inicializado.");
  }
}

void AtualizarNivelReservatorio() {
  static uint32_t ultimoNivelMs = 0;
  const uint32_t periodoNivelMs = 1000;
  uint32_t agora = millis();

  if (agora - ultimoNivelMs < periodoNivelMs) return;
  ultimoNivelMs = agora;

  if (!sensorToFOk) {
    // se nao ha sensor, tratamos como "indisponivel" (não mexe no LED)
    ultimaLeituraToFValida = false;
    nivelReservatorioOk    = true;
    digitalWrite(pinoLEDNivelBaixo, LOW);
    return;
  }

  uint16_t distancia = sensorToF.readRangeSingleMillimeters();
  if (sensorToF.timeoutOccurred()) {
    // erro de leitura
    ultimaLeituraToFValida = false;
    nivelReservatorioOk    = false;
  } else {
    ultimaLeituraToFValida = true;
    ultimaDistanciaToF     = distancia;

    bool nivelBaixo = distancia > limiteDistanciaNivelBaixoMm;
    nivelReservatorioOk = !nivelBaixo;
  }

  digitalWrite(pinoLEDNivelBaixo, nivelReservatorioOk ? LOW : HIGH);
}

: status qualitativo com base na distância medida
String ObterTextoStatusNivelRacao() {
  if (!sensorToFOk) {
    return "Sensor indisponivel";
  }

  if (!ultimaLeituraToFValida) {
    return "Sem leitura";
  }


  if (ultimaDistanciaToF <= 20) {
    return "Alto";
  } else if (ultimaDistanciaToF <= limiteDistanciaNivelBaixoMm) {
    return "Medio";
  } else {
    return "Baixo";
  }
}

String ObterClasseCSSStatusNivelRacao() {
  if (!sensorToFOk || !ultimaLeituraToFValida) {
    return "status-aten"; // amarelo (atenção)
  }

  if (ultimaDistanciaToF <= 20) {
    return "status-ok";   // verde
  } else if (ultimaDistanciaToF <= limiteDistanciaNivelBaixoMm) {
    return "status-aten"; // amarelo
  } else {
    return "status-crit"; // vermelho
  }
}

bool VerificarNivelReservatorioOk() {
  return nivelReservatorioOk;
}

// ------------------- RTC -------------------
void ImprimirHoraRTC() {
  if (!rtcOk) {
    Serial.println("RTC nao disponivel.");
    return;
  }

  DateTime now = rtc.now();
  Serial.print("RTC: ");
  Serial.print(now.year());
  Serial.print('-');
  if (now.month() < 10) Serial.print('0');
  Serial.print(now.month());
  Serial.print('-');
  if (now.day() < 10) Serial.print('0');
  Serial.print(now.day());
  Serial.print(' ');
  if (now.hour() < 10) Serial.print('0');
  Serial.print(now.hour());
  Serial.print(':');
  if (now.minute() < 10) Serial.print('0');
  Serial.print(now.minute());
  Serial.print(':');
  if (now.second() < 10) Serial.print('0');
  Serial.print(now.second());
  Serial.println();
}

void ConfigurarRTC() {
  if (!rtc.begin()) {
    Serial.println("RTC DS3231 nao encontrado. Agendamento desabilitado.");
    rtcOk = false;
    return;
  }
  rtcOk = true;

  if (rtc.lostPower()) {
    Serial.println("RTC sem data/hora valida. Ajustando para data/hora de compilacao.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  Serial.println("RTC inicializado. Hora atual:");
  ImprimirHoraRTC();
}

void InicializarRefeicoes() {
  for (int i = 0; i < maxRefeicoes; ++i) {
    refeicoes[i].habilitada = false;
    refeicoes[i].hora       = 0;
    refeicoes[i].minuto     = 0;
    refeicoes[i].massaG     = 0.0f;
  }
}

void ListarRefeicoes() {
  Serial.println("Refeicoes agendadas (1 a 5):");
  for (int i = 0; i < maxRefeicoes; ++i) {
    Serial.print("  [");
    Serial.print(i + 1);
    Serial.print("] ");
    if (!refeicoes[i].habilitada) {
      Serial.println("vazia / desabilitada");
    } else {
      Serial.print(refeicoes[i].hora);
      Serial.print(':');
      if (refeicoes[i].minuto < 10) Serial.print('0');
      Serial.print(refeicoes[i].minuto);
      Serial.print(" - ");
      Serial.print(refeicoes[i].massaG, 2);
      Serial.println(" g (habilitada)");
    }
  }
}

bool ConfigurarRefeicaoDados(int idRef, uint8_t hora, uint8_t minuto, float massaG) {
  if (idRef < 0 || idRef >= maxRefeicoes) return false;
  if (hora > 23 || minuto > 59 || massaG <= 0.0f) return false;

  refeicoes[idRef].hora       = hora;
  refeicoes[idRef].minuto     = minuto;
  refeicoes[idRef].massaG     = massaG;
  refeicoes[idRef].habilitada = true;
  return true;
}

void RemoverRefeicao(int idRef) {
  if (idRef < 0 || idRef >= maxRefeicoes) return;
  refeicoes[idRef].habilitada = false;
  refeicoes[idRef].hora       = 0;
  refeicoes[idRef].minuto     = 0;
  refeicoes[idRef].massaG     = 0.0f;
}

void HabilitarRefeicaoidRef(int idRef, bool habilitar) {
  if (idRef < 0 || idRef >= maxRefeicoes) return;
  refeicoes[idRef].habilitada = habilitar;
}

void AtualizarAgendadorRefeicoes() {
  if (!rtcOk) return;
  if (refeicaoEmProgresso) return;

  DateTime agora = rtc.now();
  int h = agora.hour();
  int m = agora.minute();

  if (h == ultimaHoraChecada && m == ultimoMinutoChecado) {
    return;
  }

  ultimaHoraChecada   = h;
  ultimoMinutoChecado = m;

  for (int i = 0; i < maxRefeicoes; ++i) {
    if (refeicoes[i].habilitada &&
        refeicoes[i].hora == h &&
        refeicoes[i].minuto == m) {

      massaRefG = refeicoes[i].massaG;
      refeicaoEmProgresso           = true;
      estadoAtual                   = EST_INICIALIZACAO;
      origemRefeicaoAtual           = ORIGEM_REFEICAO_AGENDADA; 
      slotAgendadoRefeicaoAtual     = i;                        

      Serial.print("Refeicao agendada #");
      Serial.print(i + 1);
      Serial.print(" iniciada as ");
      Serial.print(h);
      Serial.print(':');
      if (m < 10) Serial.print('0');
      Serial.print(m);
      Serial.print(" com m_ref = ");
      Serial.print(massaRefG, 2);
      Serial.println(" g.");

      break;
    }
  }
}

// ------------------- Histórico de refeições -------------------
void InicializarHistorico() {
  indiceProximoHistorico       = 0;
  quantidadeRegistrosHistorico = 0;

  for (int i = 0; i < maxRegistrosHistorico; ++i) {
    historicoRefeicoes[i].ano         = 0;
    historicoRefeicoes[i].mes         = 0;
    historicoRefeicoes[i].dia         = 0;
    historicoRefeicoes[i].hora        = 0;
    historicoRefeicoes[i].minuto      = 0;
    historicoRefeicoes[i].massaRefG   = 0.0f;
    historicoRefeicoes[i].massaFinalG = 0.0f;
    historicoRefeicoes[i].erroFinalG  = 0.0f;
    historicoRefeicoes[i].origem      = (uint8_t)ORIGEM_REFEICAO_DESCONHECIDA;
    historicoRefeicoes[i].slotAgendado = -1;
  }
}

void RegistrarHistoricoRefeicao() {
  RegistroHistorico &reg = historicoRefeicoes[indiceProximoHistorico];

  if (rtcOk) {
    DateTime agora = rtc.now();
    reg.ano    = agora.year();
    reg.mes    = agora.month();
    reg.dia    = agora.day();
    reg.hora   = agora.hour();
    reg.minuto = agora.minute();
  } else {
    reg.ano    = 0;
    reg.mes    = 0;
    reg.dia    = 0;
    reg.hora   = 0;
    reg.minuto = 0;
  }

  reg.massaRefG     = massaRefG;
  reg.massaFinalG   = massaAtualG;
  reg.erroFinalG    = erroAtualG;
  reg.origem        = (uint8_t)origemRefeicaoAtual;
  reg.slotAgendado  = slotAgendadoRefeicaoAtual;

  indiceProximoHistorico = (indiceProximoHistorico + 1) % maxRegistrosHistorico;
  if (quantidadeRegistrosHistorico < maxRegistrosHistorico) {
    quantidadeRegistrosHistorico++;
  }
}

String ObterTextoOrigemRefeicao(uint8_t origem, int8_t slotAgendado) {
  switch (origem) {
    case ORIGEM_REFEICAO_SERIAL:
      return "Manual (serial)";
    case ORIGEM_REFEICAO_BOTAO:
      return "Botao frontal";
    case ORIGEM_REFEICAO_AGENDADA: {
      String texto = "Agendada";
      if (slotAgendado >= 0) {
        texto += " (#";
        texto += String(slotAgendado + 1);
        texto += ")";
      }
      return texto;
    }
    default:
      return "Desconhecida";
  }
}

// ------------------- Botão dose fixa -------------------
void ConfigurarBotaoDose() {
  pinMode(pinoBotaoDose, INPUT_PULLUP);
}

void AtualizarBotaoDose() {
  static int estadoAnteriorLeitura = HIGH;
  static uint32_t ultimoDebounceMs = 0;
  static bool botaoPressionado = false;
  const uint32_t debounceMs = 50;

  int leitura = digitalRead(pinoBotaoDose);
  uint32_t agora = millis();

  if (leitura != estadoAnteriorLeitura) {
    ultimoDebounceMs = agora;
    estadoAnteriorLeitura = leitura;
  }

  if ((agora - ultimoDebounceMs) > debounceMs) {
    if (leitura == LOW && !botaoPressionado) {
      botaoPressionado = true;

      if (!refeicaoEmProgresso) {
        massaRefG                = massaDoseBotaoG;
        refeicaoEmProgresso      = true;
        estadoAtual              = EST_INICIALIZACAO;
        origemRefeicaoAtual      = ORIGEM_REFEICAO_BOTAO;      
        slotAgendadoRefeicaoAtual = -1;                        

        Serial.print("Refeicao via botao: m_ref = ");
        Serial.print(massaRefG, 2);
        Serial.println(" g.");
      } else {
        Serial.println("Refeicao ja em andamento; botao ignorado.");
      }
    } else if (leitura == HIGH && botaoPressionado) {
      botaoPressionado = false;
    }
  }
}

// ------------------- Máquina de estados -------------------
void AoEntrarEstado(EstadoRefeicao novoEstado) {
  switch (novoEstado) {
    case EST_ESPERA_COMANDO:
      Serial.println("[Estado] Espera por comando de refeicao.");
      break;

    case EST_INICIALIZACAO:
      Serial.println("[Estado] Inicializacao e configuracao.");
      AtualizarNivelReservatorio();
      if (!VerificarNivelReservatorioOk()) {
        Serial.println("ERRO: nivel de racao insuficiente. Refeicao cancelada.");
        refeicaoEmProgresso           = false;
        estadoAtual                   = EST_ESPERA_COMANDO;
        origemRefeicaoAtual           = ORIGEM_REFEICAO_DESCONHECIDA;
        slotAgendadoRefeicaoAtual     = -1;
        return;
      }
      AplicarEnableMotor(true);
      RealizarTara();
      break;

    case EST_PRE_DOSAGEM: {
      Serial.println("[Estado] Pre-dosagem.");

      float massaPreDosagemG = pRef * massaRefG;
      long passosPreMicro    = ConverterMassaParaPassos(massaPreDosagemG);
      if (passosPreMicro <= 0) passosPreMicro = 1;

      Serial.print("Pre-dosagem alvo: ");
      Serial.print(massaPreDosagemG, 2);
      Serial.print(" g (");
      Serial.print(pRef * 100.0f, 0);
      Serial.print("% de ");
      Serial.print(massaRefG, 2);
      Serial.println(" g).");

      Serial.print("Passos de pre-dosagem: ");
      Serial.println(passosPreMicro);

      IniciarMovimentoPassos(passosPreMicro);
      break;
    }

    case EST_ACOMODACAO:
      Serial.println("[Estado] Acomodacao.");
      tempoInicioAcomodacaoMs = millis();
      break;

    case EST_MEDICAO:
      Serial.println("[Estado] Medicao.");
      contagemAmostrasMedicao = 0;
      somaAmostrasMedicao     = 0.0;
      break;

    case EST_AVALIACAO_ERRO:
      Serial.println("[Estado] Avaliacao do erro.");
      break;

    case EST_AJUSTE_INCREMENTAL: {
      Serial.println("[Estado] Ajuste incremental.");

      float passosTeoricos = erroAtualG / massaPorPassoMotorG;
      if (passosTeoricos < 0.0f) passosTeoricos = 0.0f;

      long passosIncremento =
          (long)(ganhoIncremental * passosTeoricos + 0.5f);
      if (passosIncremento < 1) passosIncremento = 1;
      if (passosIncremento > passosMaxIncremento)
        passosIncremento = passosMaxIncremento;

      Serial.print("Passos teoricos: ");
      Serial.print(passosTeoricos, 0);
      Serial.print(", aplicados: ");
      Serial.println(passosIncremento);

      IniciarMovimentoPassos(passosIncremento);
      break;
    }

    case EST_TERMINO_REFEICAO:
      Serial.println("[Estado] Termino da refeicao.");
      AplicarEnableMotor(false);
      refeicaoEmProgresso = false;

      Serial.print("Massa final: ");
      Serial.print(massaAtualG, 2);
      Serial.println(" g.");
      Serial.print("Erro final: ");
      Serial.print(erroAtualG, 2);
      Serial.println(" g.");

      : registra no historico para a página /historico
      RegistrarHistoricoRefeicao();
      origemRefeicaoAtual       = ORIGEM_REFEICAO_DESCONHECIDA;
      slotAgendadoRefeicaoAtual = -1;
      break;
  }
}

void AtualizarEstadoInicializacao() {
  if (!taraSolicitada) {
    estadoAtual = EST_PRE_DOSAGEM;
  }
}

void AtualizarEstadoPreDosagem() {
  if (!movimentoMotorAtivo) {
    estadoAtual = EST_ACOMODACAO;
  }
}

void AtualizarEstadoAcomodacao() {
  uint32_t agora = millis();
  if (agora - tempoInicioAcomodacaoMs >= tempoAcomodacaoMs) {
    estadoAtual = EST_MEDICAO;
  }
}

void AtualizarEstadoMedicao() {
  if (novoPesoDisponivel) {
    somaAmostrasMedicao += (double)ultimoPesoG;
    contagemAmostrasMedicao++;

    if (contagemAmostrasMedicao >= amostrasMedicaoNecessarias) {
      massaAtualG = (float)(somaAmostrasMedicao /
                            (double)contagemAmostrasMedicao);

      Serial.print("Massa medida: ");
      Serial.print(massaAtualG, 2);
      Serial.println(" g.");

      estadoAtual = EST_AVALIACAO_ERRO;
    }
  }
}

void AtualizarEstadoAvaliacaoErro() {
  erroAtualG = massaRefG - massaAtualG;

  Serial.print("m_ref = ");
  Serial.print(massaRefG, 2);
  Serial.print(" g, m_atual = ");
  Serial.print(massaAtualG, 2);
  Serial.print(" g, erro = ");
  Serial.print(erroAtualG, 2);
  Serial.println(" g.");

  if (fabsf(erroAtualG) <= erroMaxG) {
    estadoAtual = EST_TERMINO_REFEICAO;
  } else if (erroAtualG > erroMaxG) {
    estadoAtual = EST_AJUSTE_INCREMENTAL;
  } else {
    Serial.println("Overshoot negativo. Encerrando refeicao.");
    estadoAtual = EST_TERMINO_REFEICAO;
  }
}

void AtualizarEstadoAjusteIncremental() {
  if (!movimentoMotorAtivo) {
    estadoAtual = EST_ACOMODACAO;
  }
}

void AtualizarMaquinaEstados() {
  if (estadoAtual != estadoAnterior) {
    estadoAnterior = estadoAtual;
    AoEntrarEstado(estadoAtual);
  }

  switch (estadoAtual) {
    case EST_ESPERA_COMANDO:
      break;
    case EST_INICIALIZACAO:
      AtualizarEstadoInicializacao();
      break;
    case EST_PRE_DOSAGEM:
      AtualizarEstadoPreDosagem();
      break;
    case EST_ACOMODACAO:
      AtualizarEstadoAcomodacao();
      break;
    case EST_MEDICAO:
      AtualizarEstadoMedicao();
      break;
    case EST_AVALIACAO_ERRO:
      AtualizarEstadoAvaliacaoErro();
      break;
    case EST_AJUSTE_INCREMENTAL:
      AtualizarEstadoAjusteIncremental();
      break;
    case EST_TERMINO_REFEICAO:
      estadoAtual = EST_ESPERA_COMANDO;
      break;
  }
}

// ------------------- Interface serial -------------------
void ImprimirAjudaSerial() {
  Serial.println();
  Serial.println("Comandos:");
  Serial.println("h = ajuda");
  Serial.println("f = iniciar refeicao com m_ref atual");
  Serial.println("m X = m_ref = X g (m 25.5)");
  Serial.println("X = linha com numero define m_ref direta");
  Serial.println("now = mostra hora atual do RTC");
  Serial.println("list = lista refeicoes (1..5)");
  Serial.println("add i hh:mm M  = adiciona/edita refeicao (ex: add 1 08:30 25.0)");
  Serial.println("edit i hh:mm M = edita refeicao");
  Serial.println("del i = remove refeicao i");
  Serial.println("on i / off i   = habilita/desabilita refeicao i");
  Serial.println("Botao pino 25: dose fixa de 10 g.");
  Serial.println();
}

void ProcessarComandoAddOuEdit(String linha, bool isEdit) {
  String resto = linha.substring(isEdit ? 5 : 4);
  resto.trim();

  int p1 = resto.indexOf(' ');
  int p2 = resto.indexOf(' ', p1 + 1);
  if (p1 < 0 || p2 < 0) {
    Serial.println("Uso: add i hh:mm massa   ou   edit i hh:mm massa");
    return;
  }

  String idRefStr = resto.substring(0, p1);
  String horaStr  = resto.substring(p1 + 1, p2);
  String massaStr = resto.substring(p2 + 1);

  idRefStr.trim();
  horaStr.trim();
  massaStr.trim();

  int idRef = idRefStr.toInt();
  if (idRef < 1 || idRef > maxRefeicoes) {
    Serial.println("Indice invalido (1 a 5).");
    return;
  }

  int colon = horaStr.indexOf(':');
  if (colon < 0) {
    Serial.println("Horario invalido. Use hh:mm.");
    return;
  }

  String hStr = horaStr.substring(0, colon);
  String mStr = horaStr.substring(colon + 1);
  hStr.trim();
  mStr.trim();

  int h = hStr.toInt();
  int m = mStr.toInt();
  float massa = massaStr.toFloat();

  if (!ConfigurarRefeicaoDados(idRef - 1, (uint8_t)h, (uint8_t)m, massa)) {
    Serial.println("Parametros invalidos.");
    return;
  }

  Serial.print(isEdit ? "Refeicao editada no slot " : "Refeicao adicionada no slot ");
  Serial.print(idRef);
  Serial.print(": ");
  Serial.print(h);
  Serial.print(':');
  if (m < 10) Serial.print('0');
  Serial.print(m);
  Serial.print(" -> ");
  Serial.print(massa, 2);
  Serial.println(" g (habilitada).");
}

void ProcessarLinhaComando(String linha) {
  linha.trim();
  if (linha.length() == 0) return;

  linha.replace(',', '.');

  if (linha.equalsIgnoreCase("h")) {
    ImprimirAjudaSerial();
    return;
  }

  if (linha.equalsIgnoreCase("f")) {
    if (!refeicaoEmProgresso) {
      refeicaoEmProgresso      = true;
      estadoAtual              = EST_INICIALIZACAO;
      origemRefeicaoAtual      = ORIGEM_REFEICAO_SERIAL;  
      slotAgendadoRefeicaoAtual = -1;                     

      Serial.print("Refeicao manual com m_ref = ");
      Serial.print(massaRefG, 2);
      Serial.println(" g.");
    } else {
      Serial.println("Refeicao em andamento.");
    }
    return;
  }

  if (linha.equalsIgnoreCase("now") || linha.equalsIgnoreCase("time")) {
    ImprimirHoraRTC();
    return;
  }

  if (linha.equalsIgnoreCase("list")) {
    ListarRefeicoes();
    return;
  }

  if (linha.startsWith("add ")) {
    ProcessarComandoAddOuEdit(linha, false);
    return;
  }

  if (linha.startsWith("edit ")) {
    ProcessarComandoAddOuEdit(linha, true);
    return;
  }

  if (linha.startsWith("del ")) {
    String resto = linha.substring(4);
    resto.trim();
    int idRef = resto.toInt();
    if (idRef < 1 || idRef > maxRefeicoes) {
      Serial.println("Indice invalido (1 a 5).");
      return;
    }
    RemoverRefeicao(idRef - 1);
    Serial.print("Refeicao removida do slot ");
    Serial.println(idRef);
    return;
  }

  if (linha.startsWith("on ")) {
    String resto = linha.substring(3);
    resto.trim();
    int idRef = resto.toInt();
    if (idRef < 1 || idRef > maxRefeicoes) {
      Serial.println("Indice invalido (1 a 5).");
      return;
    }
    HabilitarRefeicaoidRef(idRef - 1, true);
    Serial.print("Refeicao ");
    Serial.print(idRef);
    Serial.println(" habilitada.");
    return;
  }

  if (linha.startsWith("off ")) {
    String resto = linha.substring(4);
    resto.trim();
    int idRef = resto.toInt();
    if (idRef < 1 || idRef > maxRefeicoes) {
      Serial.println("Indice invalido (1 a 5).");
      return;
    }
    HabilitarRefeicaoidRef(idRef - 1, false);
    Serial.print("Refeicao ");
    Serial.print(idRef);
    Serial.println(" desabilitada.");
    return;
  }

  if (linha.length() >= 2 &&
      (linha.charAt(0) == 'm' || linha.charAt(0) == 'M')) {

    String valorStr = linha.substring(1);
    valorStr.trim();
    valorStr.replace(',', '.');

    float valor = valorStr.toFloat();
    if (valor > 0.0f) {
      massaRefG = valor;
      Serial.print("Nova massaRefG = ");
      Serial.print(massaRefG, 2);
      Serial.println(" g.");
    } else {
      Serial.println("Valor invalido para m_ref.");
    }
    return;
  }

  float valor = linha.toFloat();
  if (valor > 0.0f) {
    massaRefG = valor;
    Serial.print("Nova massaRefG = ");
    Serial.print(massaRefG, 2);
    Serial.println(" g.");
  } else {
    Serial.println("Comando invalido. Use 'h' para ajuda.");
  }
}

void LerComandosSerial() {
  static String buffer = "";

  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      if (buffer.length() > 0) {
        ProcessarLinhaComando(buffer);
        buffer = "";
      }
    } else {
      buffer += c;
    }
  }
}

// ------------------- Interface Web (HTTP) -------------------
// Página principal: agendamentos + status qualitativo do reservatório
// Ver depois: isso aqui tá pesando muito a memória do ESP, tentar algum método que o código HTML fique hospedado diretamente na Web
String GerarPaginaHTML() {
  String pagina;

  pagina += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  pagina += "<title>AANG - Refeicoes</title>";
  pagina += "<style>";
  pagina += "body{font-family:Arial, sans-serif; margin:16px;}";
  pagina += "h1{font-size:20px;margin:0;}";
  pagina += "header{display:flex;align-items:center;justify-content:space-between;margin-bottom:12px;}";
  pagina += "nav a{text-decoration:none;color:#007bff;font-size:14px;margin-left:8px;}";
  pagina += "nav a:first-child{margin-left:0;}";
  pagina += "nav a:hover{text-decoration:underline;}";
  pagina += ".status-container{margin-top:6px;margin-bottom:14px;padding:8px 12px;border-radius:8px;background:#f5f5f5;display:inline-flex;align-items:center;}";
  pagina += ".status-label{font-weight:bold;margin-right:6px;font-size:14px;}";
  pagina += ".status-badge{padding:4px 10px;border-radius:999px;font-weight:bold;font-size:13px;}";
  pagina += ".status-ok{background:#d4edda;border:1px solid #28a745;}";
  pagina += ".status-aten{background:#fff3cd;border:1px solid #ffc107;}";
  pagina += ".status-crit{background:#f8d7da;border:1px solid #dc3545;}";
  pagina += "table{border-collapse:collapse; margin-top:10px;}";
  pagina += "th,td{border:1px solid #ccc; padding:4px 8px; font-size:14px;}";
  pagina += "input,select{margin:4px 0;}";
  pagina += "</style></head><body>";

  pagina += "<header>";
  pagina += "<h1>Configuracao de refeicoes</h1>";
  pagina += "<nav><a href='/'>Agendamentos</a> | <a href='/historico'>Historico</a></nav>";
  pagina += "</header>";

  String textoStatus  = ObterTextoStatusNivelRacao();
  String classeStatus = ObterClasseCSSStatusNivelRacao();

  pagina += "<div class='status-container'>";
  pagina += "<span class='status-label'>Nivel do reservatorio:</span>";
  pagina += "<span class='status-badge ";
  pagina += classeStatus;
  pagina += "'>";
  pagina += textoStatus;
  pagina += "</span></div>";

  pagina += "<form action='/salvar' method='GET'>";
  pagina += "<label>Slot (1 a 5): </label>";
  pagina += "<select name='slot'>";
  for (int i = 1; i <= maxRefeicoes; ++i) {
    pagina += "<option value='";
    pagina += String(i);
    pagina += "'>";
    pagina += String(i);
    pagina += "</option>";
  }
  pagina += "</select><br>";

  pagina += "<label>Horario (hh:mm): </label>";
  pagina += "<input type='text' name='hora' placeholder='08:30'><br>";

  pagina += "<label>Massa (g): </label>";
  pagina += "<input type='number' name='massa' step='0.1' min='1'><br>";

  pagina += "<label>Habilitada: </label>";
  pagina += "<input type='checkbox' name='hab' value='1'><br>";

  pagina += "<input type='submit' value='Salvar'>";
  pagina += "</form>";

  pagina += "<h2>Refeicoes cadastradas</h2>";
  pagina += "<table><tr><th>Slot</th><th>Horario</th><th>Massa (g)</th><th>Estado</th><th>Acoes</th></tr>";

  for (int i = 0; i < maxRefeicoes; ++i) {
    pagina += "<tr>";

    pagina += "<td>";
    pagina += String(i + 1);
    pagina += "</td>";

    if (!refeicoes[i].habilitada) {
      pagina += "<td>-</td><td>-</td><td>Desabilitada</td>";
    } else {
      pagina += "<td>";
      if (refeicoes[i].hora < 10) pagina += "0";
      pagina += String(refeicoes[i].hora);
      pagina += ":";
      if (refeicoes[i].minuto < 10) pagina += "0";
      pagina += String(refeicoes[i].minuto);
      pagina += "</td>";

      pagina += "<td>";
      pagina += String(refeicoes[i].massaG, 1);
      pagina += "</td>";

      pagina += "<td>Habilitada</td>";
    }

    pagina += "<td>";
    pagina += "<a href='/remover?slot=";
    pagina += String(i + 1);
    pagina += "'>Remover</a>";
    pagina += "</td>";

    pagina += "</tr>";
  }

  pagina += "</table>";

  pagina += "</body></html>";
  return pagina;
}

: página /historico com as últimas refeições
String GerarPaginaHistoricoHTML() {
  String pagina;

  pagina += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  pagina += "<title>AANG - Historico</title>";
  pagina += "<style>";
  pagina += "body{font-family:Arial, sans-serif; margin:16px;}";
  pagina += "h1{font-size:20px;margin:0;}";
  pagina += "header{display:flex;align-items:center;justify-content:space-between;margin-bottom:12px;}";
  pagina += "nav a{text-decoration:none;color:#007bff;font-size:14px;margin-left:8px;}";
  pagina += "nav a:first-child{margin-left:0;}";
  pagina += "nav a:hover{text-decoration:underline;}";
  pagina += "table{border-collapse:collapse; margin-top:10px; width:100%;}";
  pagina += "th,td{border:1px solid #ccc; padding:4px 8px; font-size:14px; text-align:left;}";
  pagina += ".small{font-size:12px;color:#666;}";
  pagina += ".badge{padding:2px 6px;border-radius:8px;font-size:12px;font-weight:bold;}";
  pagina += ".origem-serial{background:#e0f7fa;border:1px solid #00acc1;}";
  pagina += ".origem-botao{background:#f3e5f5;border:1px solid #8e24aa;}";
  pagina += ".origem-agendada{background:#e8f5e9;border:1px solid #43a047;}";
  pagina += "</style></head><body>";

  pagina += "<header>";
  pagina += "<h1>Historico de refeicoes</h1>";
  pagina += "<nav><a href='/'>Agendamentos</a> | <a href='/historico'>Historico</a></nav>";
  pagina += "</header>";

  pagina += "<p class='small'>Ultimas ";
  pagina += String(quantidadeRegistrosHistorico);
  pagina += " refeicoes registradas (mais recentes primeiro).</p>";

  if (quantidadeRegistrosHistorico == 0) {
    pagina += "<p>Nenhuma refeicao registrada ate o momento.</p>";
  } else {
    pagina += "<table><tr><th>Data</th><th>Hora</th><th>Origem</th><th>Massa ref. (g)</th><th>Massa final (g)</th><th>Erro (g)</th></tr>";

    for (int n = 0; n < quantidadeRegistrosHistorico; ++n) {
      int idx = indiceProximoHistorico - 1 - n;
      if (idx < 0) idx += maxRegistrosHistorico;

      const RegistroHistorico &reg = historicoRefeicoes[idx];

      pagina += "<tr>";

      // Data
      pagina += "<td>";
      if (reg.ano == 0) {
        pagina += "-";
      } else {
        if (reg.dia < 10) pagina += "0";
        pagina += String(reg.dia);
        pagina += "/";
        if (reg.mes < 10) pagina += "0";
        pagina += String(reg.mes);
        pagina += "/";
        pagina += String(reg.ano);
      }
      pagina += "</td>";

      // Hora
      pagina += "<td>";
      if (reg.ano == 0) {
        pagina += "-";
      } else {
        if (reg.hora < 10) pagina += "0";
        pagina += String(reg.hora);
        pagina += ":";
        if (reg.minuto < 10) pagina += "0";
        pagina += String(reg.minuto);
      }
      pagina += "</td>";

      // Origem
      pagina += "<td>";
      pagina += "<span class='badge ";
      if (reg.origem == ORIGEM_REFEICAO_SERIAL) {
        pagina += "origem-serial";
      } else if (reg.origem == ORIGEM_REFEICAO_BOTAO) {
        pagina += "origem-botao";
      } else if (reg.origem == ORIGEM_REFEICAO_AGENDADA) {
        pagina += "origem-agendada";
      }
      pagina += "'>";
      pagina += ObterTextoOrigemRefeicao(reg.origem, reg.slotAgendado);
      pagina += "</span>";
      pagina += "</td>";

      // Massa ref
      pagina += "<td>";
      pagina += String(reg.massaRefG, 1);
      pagina += "</td>";

      // Massa final
      pagina += "<td>";
      pagina += String(reg.massaFinalG, 1);
      pagina += "</td>";

      // Erro
      pagina += "<td>";
      pagina += String(reg.erroFinalG, 1);
      pagina += "</td>";

      pagina += "</tr>";
    }

    pagina += "</table>";
  }

  pagina += "</body></html>";
  return pagina;
}

void TratarRaiz() {
  String pagina = GerarPaginaHTML();
  servidorWeb.send(200, "text/html", pagina);
}

void TratarSalvar() {
  if (!servidorWeb.hasArg("slot") ||
      !servidorWeb.hasArg("hora") ||
      !servidorWeb.hasArg("massa")) {
    servidorWeb.send(400, "text/plain", "Parametros incompletos.");
    return;
  }

  int slot = servidorWeb.arg("slot").toInt();
  if (slot < 1 || slot > maxRefeicoes) {
    servidorWeb.send(400, "text/plain", "Slot invalido.");
    return;
  }

  String horaStr = servidorWeb.arg("hora");
  horaStr.trim();
  int pos = horaStr.indexOf(':');
  if (pos < 0) {
    servidorWeb.send(400, "text/plain", "Horario invalido.");
    return;
  }

  String hStr = horaStr.substring(0, pos);
  String mStr = horaStr.substring(pos + 1);
  hStr.trim();
  mStr.trim();

  int   hora   = hStr.toInt();
  int   minuto = mStr.toInt();
  float massaG = servidorWeb.arg("massa").toFloat();

  bool hab = servidorWeb.hasArg("hab") && (servidorWeb.arg("hab") == "1");

  if (!ConfigurarRefeicaoDados(slot - 1, (uint8_t)hora, (uint8_t)minuto, massaG)) {
    servidorWeb.send(400, "text/plain", "Valores invalidos.");
    return;
  }

  if (!hab) {
    refeicoes[slot - 1].habilitada = false;
  }

  servidorWeb.sendHeader("Location", "/", true);
  servidorWeb.send(302, "text/plain", "");
}

void TratarRemover() {
  if (!servidorWeb.hasArg("slot")) {
    servidorWeb.send(400, "text/plain", "Slot nao informado.");
    return;
  }

  int slot = servidorWeb.arg("slot").toInt();
  if (slot < 1 || slot > maxRefeicoes) {
    servidorWeb.send(400, "text/plain", "Slot invalido.");
    return;
  }

  RemoverRefeicao(slot - 1);

  servidorWeb.sendHeader("Location", "/", true);
  servidorWeb.send(302, "text/plain", "");
}

: handler da página /historico
void TratarHistorico() {
  String pagina = GerarPaginaHistoricoHTML();
  servidorWeb.send(200, "text/html", pagina);
}

void Tratar404() {
  servidorWeb.send(404, "text/plain", "Recurso nao encontrado.");
}

void ConfigurarServidorWeb() {
  servidorWeb.on("/", HTTP_GET, TratarRaiz);
  servidorWeb.on("/salvar", HTTP_GET, TratarSalvar);
  servidorWeb.on("/remover", HTTP_GET, TratarRemover);
  servidorWeb.on("/historico", HTTP_GET, TratarHistorico); 
  servidorWeb.onNotFound(Tratar404);
  servidorWeb.begin();
}

// ------------------- Arduino setup/loop -------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("Sistema de alimentacao automatica iniciado.");

  Wire.begin(21, 22);

  // Wi-Fi em modo AP para interface web
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssidAP, senhaAP);
  IPAddress ip = WiFi.softAPIP();
  Serial.print("Ponto de acesso: ");
  Serial.println(ssidAP);
  Serial.print("IP: ");
  Serial.println(ip);

  ConfigurarServidorWeb();

  ConfigurarHX711();
  ConfigurarMotor();
  ConfigurarLEDs();
  ConfigurarBotaoDose();
  ConfigurarRTC();
  ConfigurarToF();
  InicializarRefeicoes();
  InicializarHistorico();  

  estadoAtual    = EST_ESPERA_COMANDO;
  estadoAnterior = EST_ESPERA_COMANDO;
  AoEntrarEstado(estadoAtual);

  ImprimirAjudaSerial();
}

void loop() {
  if (balanca.update()) {
    ultimoPesoG        = balanca.getData();
    novoPesoDisponivel = true;
  }

  VerificarTaraConcluida();
  AtualizarNivelReservatorio();
  AtualizarAgendadorRefeicoes();
  AtualizarMaquinaEstados();
  AtualizarMotor();
  AtualizarLEDStatusMotor();
  AtualizarBotaoDose();
  LerComandosSerial();

  servidorWeb.handleClient(); // trata requisicoes HTTP

  novoPesoDisponivel = false;
}
