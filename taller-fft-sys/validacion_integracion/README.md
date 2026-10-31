# CE1110 - Persona 4: validacion e integracion FFT

Esta carpeta **no reemplaza** los modulos de los otros integrantes. Lee los componentes existentes en `taller-fft-sys/full_system/sistema_ruido` para verificar frecuencias y ejecutar los escenarios P1, P2 y P5 de extremo a extremo.

## Ejecucion desde la raiz del repositorio

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s taller-fft-sys\validacion_integracion -p "test_*.py" -v
.\.venv\Scripts\python.exe taller-fft-sys\validacion_integracion\validacion_fft.py
```

Configuracion heredada: `fs=16000 Hz`, `N=1024` y resolucion `15.625 Hz/bin`. Cada deteccion usa eliminacion de DC, ventana Hann, FFT real y deteccion de picos independientes con tolerancia de un bin. Incluye prueba negativa de frecuencia erronea.

## Archivos generados

- `resultados/frecuencias_fft.csv`: esperado/detectado/error por modalidad de emisor.
- `resultados/metricas_integracion.csv`: SNR y atenuacion del pipeline real de Python para P1, P2 y P5.
- `resultados/espectro_P1.png`, `espectro_P2.png`, `espectro_P5.png`: evidencia espectral.
- `resultados/reporte_validacion.md`: texto de resultados para el informe grupal.
- `resultados/anexo_validacion_fft.pdf`: anexo listo para adjuntar o incorporar en el PDF grupal.

## Limitaciones importantes

- Evidencias digitales y simuladas. **No son mediciones de ADC/DAC fisicos.** Para afirmar validacion del emisor se requiere medir su salida con un receptor real/osciloscopio.
- El emisor ESP32 implementado por el equipo usa DAC de **8 bits** (GPIO25); `config.py` modela ADC/DAC de **12 bits**. Los resultados numericos de simulacion no caracterizan automaticamente el circuito analogico.
- P5 contiene interferencia solapada con una frecuencia util; no es correcto exigir que todos los filtros mejoren el SNR de todas las senales.
- Los indicadores de costo o duracion de ejecucion de Python no son medidas de rendimiento temporal del firmware.

## Entrega

El anexo de validacion debe integrarse al documento grupal; el ZIP de TEC-Digital debera incluir los programas y el PDF final del equipo, no solo este anexo.
