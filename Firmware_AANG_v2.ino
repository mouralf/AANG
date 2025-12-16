/* 
============== AANG - Alimentador Automático de Ração para Gatos ==============
Versão com conectividade Wi-Fi
Componentes principais:
(ESP32 + HX711 + TMC2209 + DS3231 + VL53L0X)
*/

// ------------------- Máquina de estados da refeição -------------------
enum EstadoRefeicao {
  EST_ESPERA_COMANDO,
  EST_INICIALIZACAO,
  EST_PRE_DOSAGEM,
  EST_ACOMODACAO,
  EST_MEDICAO,
  EST_AVALIACAO_ERRO,
  EST_AJUSTE_INCREMENTAL,
  EST_DESATOLAMENTO,      
  EST_TERMINO_REFEICAO
};

EstadoRefeicao estadoAtual    = EST_ESPERA_COMANDO;
EstadoRefeicao estadoAnterior = EST_ESPERA_COMANDO;

EstadoRefeicao ultimoEstadoDosagem = EST_PRE_DOSAGEM;     
EstadoRefeicao estadoParaRepetir   = EST_PRE_DOSAGEM;     


// ------------------- Bibliotecas -------------------
// Bibliotecas padrão
#include <Wire.h>
#include <HX711_ADC.h>
#include <AccelStepper.h>
#include <RTClib.h>
#include <VL53L0X.h>
#include <math.h>

//Bibliotecas para Wi-Fi 
#include <WiFi.h>
#include <WiFiManager.h>

// Bibliotecas para conexão com Firebase
#include <Firebase_ESP_Client.h>

#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

//Para manipulação de tempo e vetores
#include <time.h>
#include <vector>
#include <algorithm>


// ------------------- Célula de carga/HX711 -------------------
const int pinoDT  = 18;
const int pinoSCK = 19;

const float fatorInicialCalibracao = 1939.077759f; //obtido com testes com balança de precisão de cozinha
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

const float rpmControle = 30.0f;
float passosPorSegundoControle =
    rpmControle * (float)passosEfetivosVolta / 60.0f;

bool driverHabilitado        = false;
bool movimentoMotorAtivo     = false;
bool movimentoSentidoPositivo = true; 
long alvoPassosMovimento     = 0;

AccelStepper motor(AccelStepper::DRIVER, pinoSTEP, pinoDIR);

// ------------------- LEDs -------------------
const int pinoLEDStatusMotor = 13;
const int pinoLEDNivelBaixo  = 4;

// ------------------- Botão de interações manuais-------------------
const int   pinoBotaoDose   = 25;
const float massaDoseBotaoG = 10.0f;

// ------------------- Nível de ração (VL53L0X) -------------------
VL53L0X sensorToF;
bool     sensorToFOk         = false;
bool     nivelReservatorioOk = true;

// distância em mm acima da qual o nível é considerado baixo
const uint16_t limiteDistanciaNivelBaixoMm = 100;

// ------------------- RTC DS3231 -------------------
RTC_DS3231 rtc;
bool rtcOk = false;

// ------------------- Parâmetros de controle -------------------
const float massaPorPassoBaseG = 0.0307f;
const float massaPorPassoMotorG =
    massaPorPassoBaseG / (float)microPasso;

const float pRef               = 0.80f;
const float erroMaxG           = 1.0f;
const float ganhoIncremental   = 0.5f;
const long  passosMaxIncremento = 800;
const uint32_t tempoAcomodacaoMs = 2000;

const float massaRefPadraoG = 10.0f;
float massaRefG = massaRefPadraoG;

float massaAtualG = 0.0f;
float erroAtualG  = 0.0f;

const uint8_t amostrasMedicaoNecessarias = 10;
uint8_t  contagemAmostrasMedicao = 0;
double   somaAmostrasMedicao     = 0.0;

bool taraSolicitada = false;

// ------------------- Detecção de travamento -------------------
const float variacaoMinimaMassaTravamentoG = 0.02f;   // requisito minimo para identificar falha no movimento
float massaMedidaAnteriorG = 0.0f;                    // última massa "confirmada" na medição
const uint8_t maxTentativasDesatolamento = 10;
uint8_t tentativasDesatolamento = 0;

// ---------------------- Estado de detecção ----------------------

String ultimoComandoKey = ""; // último push-id processado em /comandos

struct ScheduleSig {
  String id;
  String sig; // assinatura (hora|qtd|ativo)
};

std::vector<ScheduleSig> agendamentosAnteriores;

struct AgendamentoServidorRT {
  String id;        // key do nó no Firebase
  uint8_t hora;     // 0..23
  uint8_t minuto;   // 0..59
  float massaG;     // qtd_porcao
  bool ativo;       // flg_ativo
};

static std::vector<AgendamentoServidorRT> agendamentosServidorCache;

static bool ParseHoraRefeicaoHHMM(const String &hhmm, uint8_t &h, uint8_t &m) {
  if (hhmm.length() != 5) return false;
  if (hhmm.charAt(2) != ':') return false;

  int hh = hhmm.substring(0, 2).toInt();
  int mm = hhmm.substring(3, 5).toInt();

  if (hh < 0 || hh > 23) return false;
  if (mm < 0 || mm > 59) return false;

  h = (uint8_t)hh;
  m = (uint8_t)mm;
  return true;
}

// Hora do servidor
static bool ObterHoraServidor(int &h, int &m) {
  time_t nowSec = time(nullptr);
  if (nowSec < 1700000000) return false; 
  tm t;
  localtime_r(&nowSec, &t);
  h = t.tm_hour;
  m = t.tm_min;
  return true;
}


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

// ------------------- Protótipos -------------------
void AtualizarNivelReservatorio();
bool VerificarNivelReservatorioOk();

void AplicarEnableMotor(bool habilitar);
void RealizarTara();

long ConverterMassaParaPassos(float massaG);
void IniciarMovimentoPassos(long passos);

void ProcessarLinhaComando(String linha);

void ImprimirHoraRTC();

static void RegistrarRefeicaoHistorico(float quantidade, float erro);



static uint64_t AgoraEpochMs() ;
static bool ErroEhNoInexistente(const String &err);
static int IndexPorId(const std::vector<ScheduleSig> &v, const String &id);




// ------------------- Máquina de estados -------------------

bool     refeicaoEmProgresso     = false;
uint32_t tempoInicioAcomodacaoMs = 0;


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
        refeicaoEmProgresso = false;
        estadoAtual         = EST_ESPERA_COMANDO;
        return;
      }

      AplicarEnableMotor(true);
    //   RealizarTara(); -> deixa de realizar a tara no início do processo, para evitar acumular ração no prato

      // reset de controle de travamento por refeição
      tentativasDesatolamento = 0;
      massaMedidaAnteriorG    = 0.0f;
      massaAtualG             = 0.0f;
      erroAtualG              = 0.0f;
      ultimoEstadoDosagem     = EST_PRE_DOSAGEM;
      break;

    case EST_PRE_DOSAGEM: {
      Serial.println("[Estado] Pre-dosagem.");
      ultimoEstadoDosagem = EST_PRE_DOSAGEM; 

      float massaPreDosagemG = pRef * massaRefG;
      long passosPreMicro    = ConverterMassaParaPassos(massaPreDosagemG);
      if (passosPreMicro == 0) passosPreMicro = 1;

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
      ultimoEstadoDosagem = EST_AJUSTE_INCREMENTAL;

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

    case EST_DESATOLAMENTO: { 
      Serial.println("[Estado] Desatolamento: meia volta no sentido inverso.");
      long passosMeiaVolta = (long)(passosEfetivosVolta / 2);

      Serial.print("Meia volta reversa (passos): ");
      Serial.println(passosMeiaVolta);

      IniciarMovimentoPassos(-passosMeiaVolta);
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

      RegistrarRefeicaoHistorico(massaAtualG, erroAtualG);
      break;
  }
}



// ====================== CONFIGURAÇÕES DO PROJETO ======================

// Realtime Database URL
#define FIREBASE_HOST "url_do_firebase_aqui" 

// Token legado (Database Secret)
#define FIREBASE_AUTH "<token_legado_aqui>"
//Obs: como não tem controle de acesso implementado, preferi não expor o host nem o token real aqui no código público.

// ID do dispositivo no nó /devices/<deviceId>/...
const char *deviceId = "aang-01";

// =====================================================================

FirebaseData fbdo;
FirebaseAuth firebaseAuth;
FirebaseConfig firebaseConfig;

WiFiManager wifiManager;

// Pooling
unsigned long ultimaVerificacaoMs = 0;
const unsigned long intervaloVerificacaoMs = 1500; // 1.5 s



// ---------------------- Utilitários ----------------------
static bool TentarObterHoraLocal(tm &outTm) {
  return getLocalTime(&outTm, 50);
}


static void FormatarDataHora(uint64_t tsMs, String &data, String &hora) {
  time_t sec = (time_t)(tsMs / 1000ULL);
  tm t;
  localtime_r(&sec, &t);

  char bufData[16];
  char bufHora[16];
  snprintf(bufData, sizeof(bufData), "%02d/%02d/%04d", t.tm_mday, t.tm_mon + 1, t.tm_year + 1900);
  snprintf(bufHora, sizeof(bufHora), "%02d:%02d", t.tm_hour, t.tm_min);

  data = String(bufData);
  hora = String(bufHora);
}

static void LerLinhaSerial(String &linha) {
  linha = "";
  while (linha.length() == 0) {
    if (Serial.available()) {
      linha = Serial.readStringUntil('\n');
      linha.trim();
    }
    delay(5);
  }
}


static String MontarCaminho(const char *subNo) {
  String caminho = "/devices/";
  caminho += deviceId;
  caminho += "/";
  caminho += subNo; // "comandos", "agendamentos", "history"
  return caminho;
}

static bool ErroEhNoInexistente(const String &err) {
  String e = err;
  e.toLowerCase();
  return (e.indexOf("not found") >= 0) ||
         (e.indexOf("path not exist") >= 0) ||
         (e.indexOf("requested entity was not found") >= 0);
}

// ---------------------- Detecção de AGENDAMENTOS ----------------------

static String AssinaturaAgendamento(const String &agendamentoJson) {
  FirebaseJson s;
  s.setJsonData(agendamentoJson);

  FirebaseJsonData jd;

  String hora = "";
  double qtd = 0.0;
  bool ativo = false;

  if (s.get(jd, "hora_refeicao") && jd.success) hora = jd.to<String>();
  if (s.get(jd, "qtd_porcao") && jd.success) qtd = jd.to<double>();
  if (s.get(jd, "flg_ativo") && jd.success) ativo = jd.to<bool>();

  String sig = hora;
  sig += "|";
  sig += String(qtd, 3);
  sig += "|";
  sig += (ativo ? "1" : "0");
  return sig;
}

static int IndexPorId(const std::vector<ScheduleSig> &v, const String &id) {
  for (size_t i = 0; i < v.size(); i++) {
    if (v[i].id == id) return (int)i;
  }
  return -1;
}

static void VerificarMudancasAgendamentos() {
  if (!Firebase.ready()) return;

  String path = MontarCaminho("agendamentos");

  String json;
  if (Firebase.RTDB.getJSON(&fbdo, path.c_str())) {
    json = fbdo.jsonString();
  } else {
    if (ErroEhNoInexistente(fbdo.errorReason())) json = "{}";
    else return;
  }

  json.trim();
  if (json.length() == 0 || json == "null") json = "{}";

  FirebaseJson root;
  root.setJsonData(json);

  std::vector<ScheduleSig> atuais;
  std::vector<AgendamentoServidorRT> runtime;


  size_t len = root.iteratorBegin();
  for (size_t i = 0; i < len; i++) {
    String k, v;
    int type = 0;
        root.iteratorGet(i, type, k, v);

    k.trim();
    v.trim();
    if (k.length() == 0) continue;
    if (v == "null") continue; 

    ScheduleSig s;
    s.id = k;              // id = key do nó
    s.sig = AssinaturaAgendamento(v);
    atuais.push_back(s);

    // ---------- montar cache de agendamentos executáveis ----------
    FirebaseJson sj;
    sj.setJsonData(v);

    FirebaseJsonData jd;
    String horaStr = "";
    double qtd = 0.0;
    bool ativo = false;

    if (sj.get(jd, "hora_refeicao") && jd.success) horaStr = jd.to<String>();
    if (sj.get(jd, "qtd_porcao") && jd.success) qtd = jd.to<double>();
    if (sj.get(jd, "flg_ativo") && jd.success) ativo = jd.to<bool>();

    uint8_t hh = 0, mm = 0;
    if (ativo && (qtd > 0.0) && ParseHoraRefeicaoHHMM(horaStr, hh, mm)) {
      AgendamentoServidorRT ag;
      ag.id = k;
      ag.hora = hh;
      ag.minuto = mm;
      ag.massaG = (float)qtd;
      ag.ativo = true;
      runtime.push_back(ag);
    }

  }
  root.iteratorEnd();

  for (const auto &cur : atuais) {
    int idxPrev = IndexPorId(agendamentosAnteriores, cur.id);
    if (idxPrev < 0) {
      Serial.println(F("\n=== AGENDAMENTO CRIADO ==="));
      Serial.print(F("id: "));
      Serial.println(cur.id);
      Serial.print(F("assinatura: "));
      Serial.println(cur.sig);
    } else if (agendamentosAnteriores[idxPrev].sig != cur.sig) {
      Serial.println(F("\n=== AGENDAMENTO EDITADO ==="));
      Serial.print(F("id: "));
      Serial.println(cur.id);
      Serial.print(F("antes: "));
      Serial.println(agendamentosAnteriores[idxPrev].sig);
      Serial.print(F("depois: "));
      Serial.println(cur.sig);
    }
  }

  for (const auto &prev : agendamentosAnteriores) {
    int idxCur = IndexPorId(atuais, prev.id);
    if (idxCur < 0) {
      Serial.println(F("\n=== AGENDAMENTO EXCLUIDO ==="));
      Serial.print(F("id: "));
      Serial.println(prev.id);
      Serial.print(F("assinatura antiga: "));
      Serial.println(prev.sig);
    }
  }
    std::sort(runtime.begin(), runtime.end(), [](const AgendamentoServidorRT &a, const AgendamentoServidorRT &b) {
    if (a.hora != b.hora) return a.hora < b.hora;
    return a.minuto < b.minuto;
  });

  agendamentosServidorCache.swap(runtime);

  agendamentosAnteriores = atuais;
}

// ---------------------- Listar AGENDAMENTOS (opção do menu) ----------------------

struct ScheduleView {
  String hora;
  double qtd;
  bool ativo;
};


static void ListarAgendamentosExistentes() {
  if (!Firebase.ready()) {
    Serial.println(F("[Listar] Firebase nao pronto."));
    return;
  }

  String path = MontarCaminho("agendamentos");

  String json;
  if (Firebase.RTDB.getJSON(&fbdo, path.c_str())) {
    json = fbdo.jsonString();
  } else {
    if (ErroEhNoInexistente(fbdo.errorReason())) json = "{}";
    else {
      Serial.print(F("[Listar] Erro: "));
      Serial.println(fbdo.errorReason());
      return;
    }
  }

  json.trim();
  if (json.length() == 0 || json == "null") json = "{}";

  FirebaseJson root;
  root.setJsonData(json);

  std::vector<ScheduleView> lista;

  size_t len = root.iteratorBegin();
  for (size_t i = 0; i < len; i++) {
    String k, v;
    int type = 0;
    root.iteratorGet(i, type, k, v);

    // Se o valor for "null", ignora (para evitar listar agendamentos sem dados)
    if (v == "null") continue; 

    FirebaseJson s;
    s.setJsonData(v);

    FirebaseJsonData jd;
    ScheduleView it;
    
    // Inicializa valores padrão
    it.hora = ""; 
    it.qtd = 0.0;
    it.ativo = false;

    // Tenta extrair os dados
    if (s.get(jd, "hora_refeicao") && jd.success) it.hora = jd.to<String>();
    if (s.get(jd, "qtd_porcao") && jd.success) it.qtd = jd.to<double>();
    if (s.get(jd, "flg_ativo") && jd.success) it.ativo = jd.to<bool>();

    
    // Só adiciona na lista se a hora foi preenchida corretamente (não está vazia)
    if (it.hora.length() > 0) {
        lista.push_back(it);
    }
  }
  root.iteratorEnd();

  // Ordena por hora (HH:MM)
  std::sort(lista.begin(), lista.end(), [](const ScheduleView &a, const ScheduleView &b) {
    return a.hora < b.hora;
  });

  Serial.println(F("\n=== REFEICOES (AGENDAMENTOS) EXISTENTES ==="));
  if (lista.empty()) {
    Serial.println(F("(nenhuma refeicao cadastrada)"));
    Serial.println();
    return;
  }

  for (const auto &it : lista) {
    String status = it.ativo ? "habilitado" : "desabilitado";
    Serial.print(it.hora); 
    Serial.print(F(" - "));
    Serial.print(it.qtd, 2);
    Serial.print(F(" g - "));
    Serial.println(status);
  }
  Serial.println();
}

// ---------------------- Envio de histórico de refeições ----------------------
static uint64_t AgoraEpochMs() {
  time_t nowSec = time(nullptr);
  if (nowSec < 1700000000) { 
    return (uint64_t)millis(); 
  }
  return (uint64_t)nowSec * 1000ULL + (uint64_t)(millis() % 1000);
}

// Define os tipos possíveis de origem
enum origemRefeicaoAtual {
  ORIGEM_AGENDAMENTO,   // Veio pelo agendamento
  ORIGEM_MANUAL // Veio pelo comando manual do App ou botão
};


volatile origemRefeicaoAtual origemAtual = ORIGEM_MANUAL;


static void RegistrarRefeicaoHistorico(float quantidade, float erro) {
  Serial.println(F("\n=== Enviar refeicao simulada para /history ==="));

  uint64_t tsMs = AgoraEpochMs();
  String data, hora;
  FormatarDataHora(tsMs, data, hora);

  String id = String((uint64_t)tsMs) + "-" + String((uint32_t)esp_random(), HEX);

  FirebaseJson json;
  json.set("id", id);
  json.set("ts", (int64_t)tsMs);

  // extras
  json.set("data", data);
  json.set("hora", hora);
  json.set("quantidade", String(quantidade, 2).toDouble());
  json.set("erro", erro);


  String strComando;
  if (origemAtual == ORIGEM_MANUAL) {
      strComando = "manual";
  } else {
      strComando = "agendado"; // Assume agendado se não for manual (ou ORIGEM_AGENDAMENTO)
  }
  
  json.set("comando", strComando); 
  // -------------------------------

  json.set("status", "finalizado");

  String path = MontarCaminho("history");
  path += "/";
  path += id;

  Serial.println(F("[History] Enviando..."));
  Serial.print(F("Caminho: "));
  Serial.println(path);
  Serial.print(F("Origem: "));     
  Serial.println(strComando);     
  Serial.print(F("Data/Hora: "));
  Serial.print(data);
  Serial.print(F(" "));
  Serial.println(hora);

 if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json)) {
  Serial.println(F("[History] OK!"));

  if (origemAtual == ORIGEM_MANUAL) {
    String pathCmd = MontarCaminho("comandos");

    Serial.print(F("[Comandos] Limpando no inteiro: "));
    Serial.println(pathCmd);

    if (Firebase.RTDB.deleteNode(&fbdo, pathCmd.c_str())) {
      Serial.println(F("[Comandos] OK (nó /comandos removido)."));
      ultimoComandoKey = ""; // evita reprocessar chave antiga
    } else {
      Serial.print(F("[Comandos] Falhou ao limpar: "));
      Serial.println(fbdo.errorReason());
    }
  }

  } else {
    Serial.print(F("[History] Falhou: "));
    Serial.println(fbdo.errorReason());
  }
}

// ---------------------- Wi-Fi / NTP / Firebase ----------------------

static void ConectarWiFi() {
  Serial.println(F("\n[WiFi] Conectando via WiFiManager..."));

  wifiManager.setHostname("AANG-ESP32");
  bool conectado = wifiManager.autoConnect("AANG-Setup");

  if (!conectado) {
    Serial.println(F("[WiFi] Falha ao conectar. Reiniciando..."));
    delay(1500);
    ESP.restart();
  }

  Serial.print(F("[WiFi] Conectado. IP: "));
  Serial.println(WiFi.localIP());
}


static void SincronizarNTP() {
  // GMT-3 fixo (Brasília). Sem DST.
  configTime(-3 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  tm t;
  for (int i = 0; i < 25; i++) {
    if (TentarObterHoraLocal(t)) {
      Serial.println(F("[NTP] Hora sincronizada."));
      return;
    }
    delay(80);
  }
  Serial.println(F("[NTP] Nao foi possivel sincronizar agora (vou seguir com fallback)."));
}



static void ConfigurarFirebase() {
  Serial.println(F("[Firebase] Inicializando..."));

  firebaseConfig.database_url = FIREBASE_HOST;
  firebaseConfig.signer.tokens.legacy_token = FIREBASE_AUTH;

  firebaseConfig.token_status_callback = tokenStatusCallback;

  Firebase.begin(&firebaseConfig, &firebaseAuth);
  Firebase.reconnectWiFi(true);

  Serial.println(F("[Firebase] Pronto."));
}


// ---------------------- Detecção de COMANDOS ----------------------

static bool ExtrairPrimeiraChaveEBodynode(const String &jsonRoot, String &outKey, String &outChildJson) {
  FirebaseJson root;
  root.setJsonData(jsonRoot);

  size_t len = root.iteratorBegin();
  if (len == 0) {
    root.iteratorEnd();
    return false;
  }

  String k, v;
  int type = 0;
  root.iteratorGet(0, type, k, v);
  root.iteratorEnd();

  k.trim();
  v.trim();
  if (k.length() == 0) return false;

  outKey = k;
  outChildJson = v;
  return true;
}

static void VerificarNovoComandoFeedNow() {
  if (!Firebase.ready()) return;

  String path = MontarCaminho("comandos");

  QueryFilter q;
  q.orderBy("$key");
  q.limitToLast(1);

  if (!Firebase.RTDB.getJSON(&fbdo, path.c_str(), &q)) {
    return;
  }

  String json = fbdo.jsonString();
  json.trim();
  if (json.length() == 0 || json == "null" || json == "{}") return;

  String key, body;
  if (!ExtrairPrimeiraChaveEBodynode(json, key, body)) return;

  if (key == ultimoComandoKey) return;

  ultimoComandoKey = key;

  FirebaseJson cmd;
  cmd.setJsonData(body);

  FirebaseJsonData jd;
  String tipo = "";
  double quantidade = 0.0;

  if (cmd.get(jd, "tipo") && jd.success) tipo = jd.to<String>();
  if (cmd.get(jd, "quantidade") && jd.success) quantidade = jd.to<double>();

  Serial.println(F("\n=== NOVO COMANDO EM /comandos ==="));
  Serial.print(F("push-id: "));
  Serial.println(key);
  Serial.print(F("tipo: "));
  Serial.println(tipo);
  Serial.print(F("quantidade: "));
  Serial.println(quantidade, 2);

  if (tipo == "Alimentar_Agora") {
    Serial.println(F("-> ACIONAR: alimentar agora (comando 'f' no terminal)."));
    massaRefG = massaRefPadraoG;
    ProcessarLinhaComando("f");
  }
  Serial.println();
}


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
  
  if (!driverHabilitado || passos == 0) return;

  alvoPassosMovimento      = motor.currentPosition() + passos;
  movimentoSentidoPositivo = (passos > 0);

  float velocidade = passosPorSegundoControle;
  motor.setSpeed(movimentoSentidoPositivo ? velocidade : -velocidade);

  movimentoMotorAtivo = true;
}

void AtualizarMotor() {
  if (!movimentoMotorAtivo) return;

  motor.runSpeed();

  long pos = motor.currentPosition();

  if (movimentoSentidoPositivo) {
    if (pos >= alvoPassosMovimento) {
      movimentoMotorAtivo = false;
      motor.setSpeed(0.0f);
    }
  } else {
    if (pos <= alvoPassosMovimento) {
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
    sensorToFOk = false;
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
    nivelReservatorioOk = true;
    digitalWrite(pinoLEDNivelBaixo, LOW);
    return;
  }

  uint16_t distancia = sensorToF.readRangeSingleMillimeters();
  if (sensorToF.timeoutOccurred()) {
    nivelReservatorioOk = false;
  } else {
    bool nivelBaixo = distancia > limiteDistanciaNivelBaixoMm;
    nivelReservatorioOk = !nivelBaixo;
  }

  digitalWrite(pinoLEDNivelBaixo, nivelReservatorioOk ? LOW : HIGH);
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
  if (refeicaoEmProgresso) return;

  int h = -1, m = -1;

  // 1) prioridade: hora do servidor (NTP)
  if (!ObterHoraServidor(h, m)) {
    // 2) se falhar a hora pelo servidor, pega pelo RTC
    if (!rtcOk) return;
    DateTime agora = rtc.now();
    h = agora.hour();
    m = agora.minute();
  }

  if (h == ultimaHoraChecada && m == ultimoMinutoChecado) return;

  ultimaHoraChecada   = h;
  ultimoMinutoChecado = m;

  
  for (const auto &ag : agendamentosServidorCache) {
    if (!ag.ativo) continue;

    if ((int)ag.hora == h && (int)ag.minuto == m) {
      massaRefG = ag.massaG;
      refeicaoEmProgresso = true;
      estadoAtual         = EST_INICIALIZACAO;
      origemAtual         = ORIGEM_AGENDAMENTO;

      Serial.print("Refeicao agendada (servidor) [");
      Serial.print(ag.id);
      Serial.print("] iniciada as ");
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
        massaRefG = massaDoseBotaoG;
        refeicaoEmProgresso = true;
        estadoAtual         = EST_INICIALIZACAO;
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
  if (!novoPesoDisponivel) return;

  somaAmostrasMedicao += (double)ultimoPesoG;
  contagemAmostrasMedicao++;

  if (contagemAmostrasMedicao >= amostrasMedicaoNecessarias) {
    massaAtualG = (float)(somaAmostrasMedicao /
                          (double)contagemAmostrasMedicao);

    Serial.print("Massa medida: ");
    Serial.print(massaAtualG, 2);
    Serial.println(" g.");

    // ------------------- DETECÇÃO DE TRAVAMENTO -------------------
    float delta = fabsf(massaAtualG - massaMedidaAnteriorG);
    Serial.print("Delta desde ultima medicao: ");
    Serial.print(delta, 3);
    Serial.println(" g.");

    if (delta < variacaoMinimaMassaTravamentoG) {
      tentativasDesatolamento++;

      Serial.print("TRAVAMENTO detectado (delta < ");
      Serial.print(variacaoMinimaMassaTravamentoG, 3);
      Serial.print(" g). Tentativa ");
      Serial.print(tentativasDesatolamento);
      Serial.print(" de ");
      Serial.println(maxTentativasDesatolamento);

      if (tentativasDesatolamento > maxTentativasDesatolamento) {
        Serial.println("ERRO: travamento persistente. Encerrando refeicao por seguranca.");
        estadoAtual = EST_TERMINO_REFEICAO;
        return;
      }

      // repetir o último comando de dosagem após desatolar
      estadoParaRepetir = ultimoEstadoDosagem;
      estadoAtual       = EST_DESATOLAMENTO;
      return;
    }

    // se NÃO travou: atualiza referência para a próxima medição do ciclo
    massaMedidaAnteriorG = massaAtualG;
    estadoAtual = EST_AVALIACAO_ERRO;
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

void AtualizarEstadoDesatolamento() {
  if (!movimentoMotorAtivo) {
    Serial.print("Desatolamento concluido. Repetindo ultimo comando: ");
    if (estadoParaRepetir == EST_PRE_DOSAGEM) Serial.println("PRE-DOSAGEM.");
    else Serial.println("AJUSTE INCREMENTAL.");

    estadoAtual = estadoParaRepetir;
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
    case EST_DESATOLAMENTO:
      AtualizarEstadoDesatolamento();
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

  String idRefStr   = resto.substring(0, p1);
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
      refeicaoEmProgresso = true;
      origemAtual = ORIGEM_MANUAL;
      estadoAtual         = EST_INICIALIZACAO;
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
    ListarAgendamentosExistentes();
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

// ------------------- Arduino setup/loop -------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("Sistema de alimentacao automatica iniciado.");

  ConectarWiFi();
  SincronizarNTP();
  ConfigurarFirebase();
  VerificarMudancasAgendamentos(); // já popula agendamentosServidorCache no início


  Wire.begin(21, 22);

  ConfigurarHX711();
  ConfigurarMotor();
  ConfigurarLEDs();
  ConfigurarBotaoDose();
  ConfigurarRTC();
  ConfigurarToF();
  InicializarRefeicoes();

  estadoAtual    = EST_ESPERA_COMANDO;
  estadoAnterior = EST_ESPERA_COMANDO;
  AoEntrarEstado(estadoAtual);

  ImprimirAjudaSerial();
}

void loop() {

 if (millis() - ultimaVerificacaoMs >= intervaloVerificacaoMs && !refeicaoEmProgresso) {
    ultimaVerificacaoMs = millis();
    VerificarNovoComandoFeedNow();
    VerificarMudancasAgendamentos();
  }

  if (balanca.update()) {
    ultimoPesoG        = balanca.getData();
    novoPesoDisponivel = true;
  }

//   VerificarTaraConcluida(); //deixa de tarar a balança no início do processo
  AtualizarNivelReservatorio();
  AtualizarAgendadorRefeicoes();
  AtualizarMaquinaEstados();
  AtualizarMotor();
  AtualizarLEDStatusMotor();
  AtualizarBotaoDose();
  LerComandosSerial();

  novoPesoDisponivel = false;
}
