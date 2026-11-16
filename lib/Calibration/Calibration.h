#ifndef CALIBRACION_H
#define CALIBRACION_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Preferences.h>

enum RangoCorrientePGA : uint8_t {
    RANGO_400MA  = 0, // PGA +/- 40mV  -> paso 0.1 mA
    RANGO_800MA  = 1, // PGA +/- 80mV  -> paso 0.2 mA
    RANGO_1600MA = 2, // PGA +/- 160mV -> paso 0.4 mA
    RANGO_3200MA = 3  // PGA +/- 320mV -> paso 0.8 mA
};

struct PuntoCalibracion {
    float lecturaSensor;
    float lecturaMultimetro;
};

struct ParametrosCalibracion {
    float pendienteCorriente;
    float interceptoCorriente;
    float r2Corriente;
    float pendienteVoltaje;
    float interceptoVoltaje;
    uint8_t rangoPGA;
    bool calibrado;
};

struct LecturaCalibrada {
    float voltajeBus_V;
    float voltajeBusCrudo_V;
    float voltajeShunt_mV;
    float corrienteCruda_mA;
    float corrienteCalibrada_mA;
    float potencia_mW;
    bool hayAlerta;
    String motivoAlerta;
};

class CalibradorINA219 {
private:
    static constexpr uint8_t MAX_PUNTOS = 10;
    static constexpr uint8_t DIRECCION_INA219 = 0x40;

    uint8_t _pinBoton;
    uint8_t _pinLed;
    float _rShunt;

    ParametrosCalibracion _params;
    Preferences _prefs;

    // Puntos de calibración multipunto
    PuntoCalibracion _puntosI[MAX_PUNTOS];
    uint8_t _numPuntosI;

    PuntoCalibracion _puntosV[MAX_PUNTOS];
    uint8_t _numPuntosV;

    // Captura congelada para contraste con multímetro
    bool _esperandoValorMultimetro;
    float _ultimaCapturaCrudaI;
    float _ultimaCapturaCrudaV;

    // Control de botón físico (GPIO 33)
    bool _estadoAnteriorBoton;
    unsigned long _tiempoPresionado;
    bool _accionLargaEjecutada;

    // Gestión exclusiva de alertas para el LED (GPIO 5)
    float _umbralCorrienteMax_mA;
    float _umbralVoltajeMax_V;
    float _toleranciaMaxPct;
    bool _alertaCalibracion;
    String _motivoAlertaCalibracion;

    void configurarHardwareAltaPrecision(Adafruit_INA219 &ina, RangoCorrientePGA rango);
    void calcularRegresionLineal(const PuntoCalibracion* puntos, uint8_t n, float &m, float &b, float &r2);
    void actualizarEstadoLed(bool alertaTiempoReal);
    bool verificarConexionI2C();
    bool verificarOverflowPGA();
    void guardarEnFlash();
    void cargarDeFlash();
    void procesarSerial(Adafruit_INA219 &ina);

public:
    CalibradorINA219(uint8_t pinBoton, uint8_t pinLed, float rShunt = 0.1f);

    bool iniciar(Adafruit_INA219 &ina, RangoCorrientePGA rangoInicial = RANGO_800MA);
    void actualizar(Adafruit_INA219 &ina);

    // Métodos de calibración asistida por multímetro
    void capturarPuntoMuestra(Adafruit_INA219 &ina, uint16_t muestras = 100);
    void registrarMultimetroCorriente(float corrienteMultimetro_mA);
    void registrarMultimetroVoltaje(float voltajeMultimetro_V);
    void calibrarCeroRapido(Adafruit_INA219 &ina, uint16_t muestras = 150);
    void cambiarRangoPGA(Adafruit_INA219 &ina, RangoCorrientePGA nuevoRango);
    void borrarPuntosTemporales();
    void restablecerFabrica(Adafruit_INA219 &ina);
    void limpiarAlertaCalibracion();

    // Configuración de umbrales de alerta
    void configurarUmbralCorriente(float max_mA);
    void configurarUmbralVoltaje(float max_V);
    void configurarToleranciaMaxima(float maxPct);

    // Lectura y reportes
    LecturaCalibrada leer(Adafruit_INA219 &ina);
    void imprimirReporteError();
    void imprimirMenuAyuda();
};

#endif