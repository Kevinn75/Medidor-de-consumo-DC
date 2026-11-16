#include "Calibration.h"
#include <math.h>

CalibradorINA219::CalibradorINA219(uint8_t pinBoton, uint8_t pinLed, float rShunt) {
    _pinBoton = pinBoton;
    _pinLed = pinLed;
    _rShunt = rShunt;

    _params.pendienteCorriente = 1.0f;
    _params.interceptoCorriente = 0.0f;
    _params.r2Corriente = 1.0f;
    _params.pendienteVoltaje = 1.0f;
    _params.interceptoVoltaje = 0.0f;
    _params.rangoPGA = RANGO_800MA;
    _params.calibrado = false;

    _numPuntosI = 0;
    _numPuntosV = 0;
    _esperandoValorMultimetro = false;
    _ultimaCapturaCrudaI = 0.0f;
    _ultimaCapturaCrudaV = 0.0f;

    _estadoAnteriorBoton = HIGH;
    _tiempoPresionado = 0;
    _accionLargaEjecutada = false;

    // Umbrales de alerta por defecto (bus nominal 10 V, rango 800 mA, tolerancia 1%)
    _umbralCorrienteMax_mA = 780.0f;
    _umbralVoltajeMax_V = 11.5f;
    _toleranciaMaxPct = 1.0f;
    _alertaCalibracion = false;
    _motivoAlertaCalibracion = "";
}

bool CalibradorINA219::iniciar(Adafruit_INA219 &ina, RangoCorrientePGA rangoInicial) {
    pinMode(_pinBoton, INPUT_PULLUP);
    pinMode(_pinLed, OUTPUT);
    digitalWrite(_pinLed, LOW); // LED apagado por defecto en estado normal

    if (!ina.begin()) {
        // ALERTA CRÍTICA: Sensor no detectado -> Encender LED fijo
        digitalWrite(_pinLed, HIGH);
        Serial.println(F("[ALERTA CRITICA] INA219 no encontrado en I2C (0x40). Revisa SDA=21 y SCL=22."));
        return false;
    }

    cargarDeFlash();
    RangoCorrientePGA rangoActivo = _params.calibrado ? (RangoCorrientePGA)_params.rangoPGA : rangoInicial;
    configurarHardwareAltaPrecision(ina, rangoActivo);

    digitalWrite(_pinLed, LOW); // Sistema listo sin alertas -> LED apagado
    imprimirMenuAyuda();
    imprimirReporteError();

    return true;
}

void CalibradorINA219::configurarHardwareAltaPrecision(Adafruit_INA219 &ina, RangoCorrientePGA rango) {
    _params.rangoPGA = (uint8_t)rango;

    if (rango == RANGO_400MA) {
        ina.setCalibration_16V_400mA();
        _umbralCorrienteMax_mA = 390.0f;
    } else {
        ina.setCalibration_32V_2A();
        if (rango == RANGO_800MA)       _umbralCorrienteMax_mA = 780.0f;
        else if (rango == RANGO_1600MA) _umbralCorrienteMax_mA = 1560.0f;
        else                            _umbralCorrienteMax_mA = 3100.0f;
    }

    // Registro 0x00: Rango Bus 16V (óptimo para 10V) + Promedio interno 128 muestras (BADC y SADC)
    uint16_t pgBits = 0x0800;
    switch (rango) {
        case RANGO_400MA:  pgBits = 0x0000; break;
        case RANGO_800MA:  pgBits = 0x0800; break;
        case RANGO_1600MA: pgBits = 0x1000; break;
        case RANGO_3200MA: pgBits = 0x1800; break;
    }

    uint16_t configReg = 0x0000 | pgBits | 0x0780 | 0x0078 | 0x0007;

    Wire.beginTransmission(DIRECCION_INA219);
    Wire.write(0x00);
    Wire.write((configReg >> 8) & 0xFF);
    Wire.write(configReg & 0xFF);
    Wire.endTransmission();

    delay(150);
}

void CalibradorINA219::actualizar(Adafruit_INA219 &ina) {
    // 1. Lectura del botón físico en GPIO 33
    bool estadoActual = digitalRead(_pinBoton);

    if (_estadoAnteriorBoton == HIGH && estadoActual == LOW) {
        _tiempoPresionado = millis();
        _accionLargaEjecutada = false;
    }

    // Pulsación larga (> 2.5 s): Restablecer fábrica
    if (estadoActual == LOW && !_accionLargaEjecutada) {
        if (millis() - _tiempoPresionado >= 2500) {
            restablecerFabrica(ina);
            _accionLargaEjecutada = true;
        }
    }

    // Pulsación corta (< 2.5 s): Capturar punto de muestra estable
    if (_estadoAnteriorBoton == LOW && estadoActual == HIGH) {
        unsigned long duracion = millis() - _tiempoPresionado;
        if (duracion >= 50 && duracion < 2500 && !_accionLargaEjecutada) {
            capturarPuntoMuestra(ina, 100);
        }
    }

    _estadoAnteriorBoton = estadoActual;

    // 2. Procesar comandos del Monitor Serial
    procesarSerial(ina);
}

void CalibradorINA219::procesarSerial(Adafruit_INA219 &ina) {
    if (!Serial.available()) return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "CAP") {
        capturarPuntoMuestra(ina, 100);
    }
    else if (cmd.startsWith("MI:")) {
        registrarMultimetroCorriente(cmd.substring(3).toFloat());
    }
    else if (cmd.startsWith("MV:")) {
        registrarMultimetroVoltaje(cmd.substring(3).toFloat());
    }
    else if (cmd == "CERO") {
        calibrarCeroRapido(ina, 150);
    }
    else if (cmd.startsWith("RANGO:")) {
        int r = cmd.substring(6).toInt();
        if (r == 400)       cambiarRangoPGA(ina, RANGO_400MA);
        else if (r == 800)  cambiarRangoPGA(ina, RANGO_800MA);
        else if (r == 1600) cambiarRangoPGA(ina, RANGO_1600MA);
        else if (r == 3200) cambiarRangoPGA(ina, RANGO_3200MA);
        else Serial.println(F("[AVISO] Rangos validos: RANGO:400, RANGO:800, RANGO:1600, RANGO:3200"));
    }
    else if (cmd.startsWith("AL_I:")) {
        configurarUmbralCorriente(cmd.substring(5).toFloat());
    }
    else if (cmd.startsWith("AL_V:")) {
        configurarUmbralVoltaje(cmd.substring(5).toFloat());
    }
    else if (cmd.startsWith("TOL:")) {
        configurarToleranciaMaxima(cmd.substring(4).toFloat());
    }
    else if (cmd == "ACK") {
        limpiarAlertaCalibracion();
    }
    else if (cmd == "LIMPIAR") {
        borrarPuntosTemporales();
    }
    else if (cmd == "RESET") {
        restablecerFabrica(ina);
    }
    else if (cmd == "INFO") {
        imprimirReporteError();
    }
    else if (cmd == "AYUDA") {
        imprimirMenuAyuda();
    }
}

void CalibradorINA219::capturarPuntoMuestra(Adafruit_INA219 &ina, uint16_t muestras) {
    if (!verificarConexionI2C()) {
        _alertaCalibracion = true;
        _motivoAlertaCalibracion = "Sin comunicacion I2C con INA219";
        actualizarEstadoLed(true);
        Serial.println(F("[ALERTA] No hay respuesta del INA219 al intentar capturar muestra."));
        return;
    }

    Serial.println(F("\n[CAPTURA] Promediando 100 muestras del sensor... Manten la carga estable."));
    double sumaI = 0.0;
    double sumaV = 0.0;

    for (uint16_t i = 0; i < muestras; i++) {
        sumaI += ina.getCurrent_mA();
        sumaV += ina.getBusVoltage_V();
        delay(10); // El LED permanece APAGADO durante muestreo normal
    }

    _ultimaCapturaCrudaI = (float)(sumaI / muestras);
    _ultimaCapturaCrudaV = (float)(sumaV / muestras);
    _esperandoValorMultimetro = true;
    limpiarAlertaCalibracion();

    Serial.println(F("=========================================================="));
    Serial.print(F("  LECTURA CONGELADA -> I_cruda = "));
    Serial.print(_ultimaCapturaCrudaI, 3);
    Serial.print(F(" mA | V_crudo = "));
    Serial.print(_ultimaCapturaCrudaV, 4);
    Serial.println(F(" V"));
    Serial.println(F("  Ingresa la lectura del multimetro patron:"));
    Serial.println(F("    -> MI:<valor_mA>   (Ej: MI:148.65 o MI:0)"));
    Serial.println(F("    -> MV:<valor_V>    (Ej: MV:10.012)"));
    Serial.println(F("==========================================================\n"));
}

void CalibradorINA219::registrarMultimetroCorriente(float corrienteMultimetro_mA) {
    if (!_esperandoValorMultimetro) {
        _alertaCalibracion = true;
        _motivoAlertaCalibracion = "Intento de registro sin captura previa";
        actualizarEstadoLed(true);
        Serial.println(F("[ALERTA] Presiona primero el boton (GPIO 33) o envia 'CAP' antes de ingresar MI."));
        return;
    }
    if (_numPuntosI >= MAX_PUNTOS) {
        _alertaCalibracion = true;
        _motivoAlertaCalibracion = "Tabla de puntos llena (Max 10)";
        actualizarEstadoLed(true);
        Serial.println(F("[ALERTA] Maximo de 10 puntos alcanzado. Envia 'LIMPIAR' para reiniciar."));
        return;
    }

    _puntosI[_numPuntosI].lecturaSensor = _ultimaCapturaCrudaI;
    _puntosI[_numPuntosI].lecturaMultimetro = corrienteMultimetro_mA;
    _numPuntosI++;
    _esperandoValorMultimetro = false;

    Serial.print(F("[OK] Punto #"));
    Serial.print(_numPuntosI);
    Serial.print(F(" registrado: Sensor = "));
    Serial.print(_ultimaCapturaCrudaI, 3);
    Serial.print(F(" mA  <--->  Multimetro = "));
    Serial.print(corrienteMultimetro_mA, 3);
    Serial.println(F(" mA"));

    if (_numPuntosI == 1) {
        _params.pendienteCorriente = 1.0f;
        _params.interceptoCorriente = corrienteMultimetro_mA - _ultimaCapturaCrudaI;
        _params.r2Corriente = 1.0f;
        limpiarAlertaCalibracion();
        Serial.println(F("  -> Captura al menos 1 punto adicional con distinta carga para ajustar pendiente y offset."));
    } else {
        calcularRegresionLineal(_puntosI, _numPuntosI,
                                _params.pendienteCorriente,
                                _params.interceptoCorriente,
                                _params.r2Corriente);

        // Verificar si algún punto supera la tolerancia máxima configurada
        bool fueraDeTolerancia = false;
        for (uint8_t i = 0; i < _numPuntosI; i++) {
            float mult = _puntosI[i].lecturaMultimetro;
            if (fabs(mult) > 1.0f) {
                float cal = (_puntosI[i].lecturaSensor * _params.pendienteCorriente) + _params.interceptoCorriente;
                float errPct = (fabs(cal - mult) / fabs(mult)) * 100.0f;
                if (errPct > _toleranciaMaxPct) {
                    fueraDeTolerancia = true;
                }
            }
        }

        if (fueraDeTolerancia) {
            _alertaCalibracion = true;
            _motivoAlertaCalibracion = "Error % supera tolerancia maxima";
            actualizarEstadoLed(true);
            Serial.print(F("[ALERTA] Uno o mas puntos superan la tolerancia permitida de "));
            Serial.print(_toleranciaMaxPct, 2);
            Serial.println(F("%. Revisa la tabla y envia 'LIMPIAR' si hubo error de lectura."));
        } else {
            limpiarAlertaCalibracion();
        }
    }

    _params.calibrado = true;
    guardarEnFlash();
    imprimirReporteError();
}

void CalibradorINA219::registrarMultimetroVoltaje(float voltajeMultimetro_V) {
    if (!_esperandoValorMultimetro) {
        _alertaCalibracion = true;
        _motivoAlertaCalibracion = "Intento de registro V sin captura previa";
        actualizarEstadoLed(true);
        Serial.println(F("[ALERTA] Presiona primero el boton (GPIO 33) o envia 'CAP' antes de ingresar MV."));
        return;
    }
    if (_numPuntosV >= MAX_PUNTOS) {
        _alertaCalibracion = true;
        _motivoAlertaCalibracion = "Tabla de puntos V llena";
        actualizarEstadoLed(true);
        return;
    }

    _puntosV[_numPuntosV].lecturaSensor = _ultimaCapturaCrudaV;
    _puntosV[_numPuntosV].lecturaMultimetro = voltajeMultimetro_V;
    _numPuntosV++;
    _esperandoValorMultimetro = false;

    if (_numPuntosV == 1) {
        if (_ultimaCapturaCrudaV > 0.1f) {
            _params.pendienteVoltaje = voltajeMultimetro_V / _ultimaCapturaCrudaV;
            _params.interceptoVoltaje = 0.0f;
        }
    } else {
        float r2V = 1.0f;
        calcularRegresionLineal(_puntosV, _numPuntosV,
                                _params.pendienteVoltaje,
                                _params.interceptoVoltaje,
                                r2V);
    }

    _params.calibrado = true;
    limpiarAlertaCalibracion();
    guardarEnFlash();

    Serial.print(F("[OK] Voltaje calibrado -> V_real = "));
    Serial.print(_params.pendienteVoltaje, 6);
    Serial.print(F(" * V_sensor + ("));
    Serial.print(_params.interceptoVoltaje, 4);
    Serial.println(F(" V)\n"));
}

void CalibradorINA219::calcularRegresionLineal(const PuntoCalibracion* puntos, uint8_t n, float &m, float &b, float &r2) {
    double sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumX2 = 0.0, sumY2 = 0.0;

    for (uint8_t i = 0; i < n; i++) {
        double x = puntos[i].lecturaSensor;
        double y = puntos[i].lecturaMultimetro;
        sumX  += x;
        sumY  += y;
        sumXY += x * y;
        sumX2 += x * x;
        sumY2 += y * y;
    }

    double denom = (n * sumX2) - (sumX * sumX);
    if (fabs(denom) < 1e-9) {
        m = 1.0f;
        b = 0.0f;
        r2 = 0.0f;
        _alertaCalibracion = true;
        _motivoAlertaCalibracion = "Puntos identicos en regresion lineal";
        actualizarEstadoLed(true);
        return;
    }

    double slope = ((n * sumXY) - (sumX * sumY)) / denom;
    double intercept = (sumY - slope * sumX) / n;

    double numR = (n * sumXY) - (sumX * sumY);
    double denR = sqrt(((n * sumX2) - (sumX * sumX)) * ((n * sumY2) - (sumY * sumY)));
    double r = (denR > 1e-9) ? (numR / denR) : 0.0;

    m = (float)slope;
    b = (float)intercept;
    r2 = (float)(r * r);
}

void CalibradorINA219::calibrarCeroRapido(Adafruit_INA219 &ina, uint16_t muestras) {
    Serial.println(F("\n[CERO] Midiendo offset en vacio (carga desconectada)..."));
    double sumaI = 0.0;
    for (uint16_t i = 0; i < muestras; i++) {
        sumaI += ina.getCurrent_mA();
        delay(10);
    }
    float crudoCero = (float)(sumaI / muestras);
    _params.interceptoCorriente = -(_params.pendienteCorriente * crudoCero);
    _params.calibrado = true;
    limpiarAlertaCalibracion();
    guardarEnFlash();

    Serial.print(F("[OK] Tara de cero aplicada. Nuevo intercepto b_I = "));
    Serial.print(_params.interceptoCorriente, 4);
    Serial.println(F(" mA\n"));
}

void CalibradorINA219::cambiarRangoPGA(Adafruit_INA219 &ina, RangoCorrientePGA nuevoRango) {
    configurarHardwareAltaPrecision(ina, nuevoRango);
    borrarPuntosTemporales();
    guardarEnFlash();
    Serial.print(F("[CONFIG] Rango PGA actualizado. Umbral alerta corriente: "));
    Serial.print(_umbralCorrienteMax_mA, 1);
    Serial.println(F(" mA"));
}

void CalibradorINA219::borrarPuntosTemporales() {
    _numPuntosI = 0;
    _numPuntosV = 0;
    _esperandoValorMultimetro = false;
    limpiarAlertaCalibracion();
    Serial.println(F("[INFO] Tabla temporal de puntos borrada."));
}

void CalibradorINA219::restablecerFabrica(Adafruit_INA219 &ina) {
    _params.pendienteCorriente = 1.0f;
    _params.interceptoCorriente = 0.0f;
    _params.r2Corriente = 1.0f;
    _params.pendienteVoltaje = 1.0f;
    _params.interceptoVoltaje = 0.0f;
    _params.calibrado = false;
    borrarPuntosTemporales();
    guardarEnFlash();

    Serial.println(F("\n[RESET] Calibracion restablecida a valores de fabrica (m=1, b=0).\n"));
}

void CalibradorINA219::limpiarAlertaCalibracion() {
    _alertaCalibracion = false;
    _motivoAlertaCalibracion = "";
    actualizarEstadoLed(false);
}

void CalibradorINA219::configurarUmbralCorriente(float max_mA) {
    if (max_mA > 0.0f) {
        _umbralCorrienteMax_mA = max_mA;
        Serial.print(F("[CONFIG] Nuevo umbral de alerta de corriente: "));
        Serial.print(_umbralCorrienteMax_mA, 2);
        Serial.println(F(" mA"));
    }
}

void CalibradorINA219::configurarUmbralVoltaje(float max_V) {
    if (max_V > 0.0f) {
        _umbralVoltajeMax_V = max_V;
        Serial.print(F("[CONFIG] Nuevo umbral de alerta de voltaje: "));
        Serial.print(_umbralVoltajeMax_V, 2);
        Serial.println(F(" V"));
    }
}

void CalibradorINA219::configurarToleranciaMaxima(float maxPct) {
    if (maxPct > 0.0f) {
        _toleranciaMaxPct = maxPct;
        Serial.print(F("[CONFIG] Nueva tolerancia maxima permitida: "));
        Serial.print(_toleranciaMaxPct, 3);
        Serial.println(F(" %"));
    }
}

bool CalibradorINA219::verificarConexionI2C() {
    Wire.beginTransmission(DIRECCION_INA219);
    return (Wire.endTransmission() == 0);
}

bool CalibradorINA219::verificarOverflowPGA() {
    // Leer registro 0x02 (Bus Voltage Register); Bit 0 = OVF (Math Overflow Flag)
    Wire.beginTransmission(DIRECCION_INA219);
    Wire.write(0x02);
    if (Wire.endTransmission() != 0) return true;

    Wire.requestFrom(DIRECCION_INA219, (uint8_t)2);
    if (Wire.available() < 2) return true;

    uint16_t rawBus = (Wire.read() << 8) | Wire.read();
    return (rawBus & 0x0001) != 0;
}

void CalibradorINA219::actualizarEstadoLed(bool alertaTiempoReal) {
    // El LED en GPIO 5 SOLO se enciende cuando hay una alerta activa
    bool encenderLed = alertaTiempoReal || _alertaCalibracion;
    digitalWrite(_pinLed, encenderLed ? HIGH : LOW);
}

LecturaCalibrada CalibradorINA219::leer(Adafruit_INA219 &ina) {
    LecturaCalibrada d;
    d.hayAlerta = false;
    d.motivoAlerta = "";

    // 1. Verificar fallo de bus I2C en tiempo real
    if (!verificarConexionI2C()) {
        d.voltajeBus_V = 0.0f;
        d.voltajeBusCrudo_V = 0.0f;
        d.voltajeShunt_mV = 0.0f;
        d.corrienteCruda_mA = 0.0f;
        d.corrienteCalibrada_mA = 0.0f;
        d.potencia_mW = 0.0f;
        d.hayAlerta = true;
        d.motivoAlerta = "DESCONEXION I2C INA219";
        actualizarEstadoLed(true);
        return d;
    }

    d.voltajeBusCrudo_V = ina.getBusVoltage_V();
    d.voltajeShunt_mV   = ina.getShuntVoltage_mV();
    d.corrienteCruda_mA = ina.getCurrent_mA();

    d.voltajeBus_V = (d.voltajeBusCrudo_V * _params.pendienteVoltaje) + _params.interceptoVoltaje;
    d.corrienteCalibrada_mA = (d.corrienteCruda_mA * _params.pendienteCorriente) + _params.interceptoCorriente;

    if (fabs(d.corrienteCalibrada_mA) < 0.15f) {
        d.corrienteCalibrada_mA = 0.0f;
    }

    d.potencia_mW = d.voltajeBus_V * d.corrienteCalibrada_mA;

    // 2. Evaluar condiciones de alerta eléctrica en tiempo real
    if (verificarOverflowPGA()) {
        d.hayAlerta = true;
        d.motivoAlerta = "SATURACION PGA (Aumenta RANGO)";
    } else if (fabs(d.corrienteCalibrada_mA) > _umbralCorrienteMax_mA) {
        d.hayAlerta = true;
        d.motivoAlerta = "SOBRECORRIENTE (> " + String(_umbralCorrienteMax_mA, 0) + " mA)";
    } else if (d.voltajeBus_V > _umbralVoltajeMax_V) {
        d.hayAlerta = true;
        d.motivoAlerta = "SOBREVOLTAJE (> " + String(_umbralVoltajeMax_V, 1) + " V)";
    } else if (_alertaCalibracion) {
        d.hayAlerta = true;
        d.motivoAlerta = _motivoAlertaCalibracion;
    }

    // Actualizar GPIO 5: HIGH solo si hay alerta, LOW en operación normal
    actualizarEstadoLed(d.hayAlerta);

    return d;
}

void CalibradorINA219::imprimirReporteError() {
    Serial.println(F("\n================ REPORTE DE CALIBRACION Y TOLERANCIA ================"));
    Serial.print(F("Estado:        "));
    Serial.println(_params.calibrado ? F("CALIBRADO (Guardado en Flash ESP32)") : F("SIN CALIBRAR"));
    Serial.print(F("Ecuacion I:    I_real = "));
    Serial.print(_params.pendienteCorriente, 6);
    Serial.print(F(" * I_sensor + ("));
    Serial.print(_params.interceptoCorriente, 4);
    Serial.println(F(" mA)"));
    Serial.print(F("Linealidad:    R^2 = "));
    Serial.println(_params.r2Corriente, 6);
    Serial.print(F("Ecuacion V:    V_real = "));
    Serial.print(_params.pendienteVoltaje, 6);
    Serial.print(F(" * V_sensor + ("));
    Serial.print(_params.interceptoVoltaje, 4);
    Serial.println(F(" V)"));
    Serial.printf("Umbrales LED:  I_max = %.1f mA | V_max = %.2f V | Tolerancia = %.2f%%\n",
                  _umbralCorrienteMax_mA, _umbralVoltajeMax_V, _toleranciaMaxPct);

    if (_numPuntosI > 0) {
        Serial.println(F("\n--- Verificacion vs Multimetro Patron (Corriente) ---"));
        Serial.println(F("Pt | Multimetro (mA) | Crudo (mA) | Calibrado (mA) | Error Abs | Error %"));
        for (uint8_t i = 0; i < _numPuntosI; i++) {
            float mult = _puntosI[i].lecturaMultimetro;
            float crud = _puntosI[i].lecturaSensor;
            float cal  = (crud * _params.pendienteCorriente) + _params.interceptoCorriente;
            float errAbs = cal - mult;
            float errPct = (fabs(mult) > 0.5f) ? (fabs(errAbs) / fabs(mult)) * 100.0f : 0.0f;

            Serial.printf(" %d | %15.3f | %10.3f | %14.3f | %+8.3f | %6.3f%%\n",
                          i + 1, mult, crud, cal, errAbs, errPct);
        }
    }
    Serial.println(F("=====================================================================\n"));
}

void CalibradorINA219::imprimirMenuAyuda() {
    Serial.println(F("\n--- COMANDOS DE CALIBRACION Y ALERTAS ---"));
    Serial.println(F("  Boton GPIO 33 (corto) o 'CAP' -> Congela y promedia 100 muestras del sensor"));
    Serial.println(F("  MI:125.40                     -> Registra lectura del multimetro en mA"));
    Serial.println(F("  MV:10.015                     -> Registra lectura del multimetro en V"));
    Serial.println(F("  CERO                          -> Tara rapida de cero (carga desconectada)"));
    Serial.println(F("  RANGO:400 / 800 / 1600 / 3200 -> Cambia ganancia PGA interna"));
    Serial.println(F("  AL_I:500  /  AL_V:11.0        -> Ajusta umbrales que encienden el LED de alerta"));
    Serial.println(F("  TOL:0.5                       -> Tolerancia max (%) antes de alerta de calibracion"));
    Serial.println(F("  ACK                           -> Apaga alerta de calibracion latente"));
    Serial.println(F("  LIMPIAR                       -> Borra tabla temporal de puntos"));
    Serial.println(F("  INFO                          -> Muestra ecuaciones y tabla de error %"));
    Serial.println(F("  Boton GPIO 33 (>2.5s) o RESET -> Borra calibracion de Flash\n"));
}

void CalibradorINA219::guardarEnFlash() {
    _prefs.begin("ina219_metro", false);
    _prefs.putFloat("m_i", _params.pendienteCorriente);
    _prefs.putFloat("b_i", _params.interceptoCorriente);
    _prefs.putFloat("r2_i", _params.r2Corriente);
    _prefs.putFloat("m_v", _params.pendienteVoltaje);
    _prefs.putFloat("b_v", _params.interceptoVoltaje);
    _prefs.putUChar("pga", _params.rangoPGA);
    _prefs.putBool("cal", _params.calibrado);
    _prefs.end();
}

void CalibradorINA219::cargarDeFlash() {
    _prefs.begin("ina219_metro", true);
    _params.pendienteCorriente  = _prefs.getFloat("m_i", 1.0f);
    _params.interceptoCorriente = _prefs.getFloat("b_i", 0.0f);
    _params.r2Corriente         = _prefs.getFloat("r2_i", 1.0f);
    _params.pendienteVoltaje    = _prefs.getFloat("m_v", 1.0f);
    _params.interceptoVoltaje   = _prefs.getFloat("b_v", 0.0f);
    _params.rangoPGA            = _prefs.getUChar("pga", RANGO_800MA);
    _params.calibrado           = _prefs.getBool("cal", false);
    _prefs.end();
}