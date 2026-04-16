/*
 * converter_app.h - Unit Converter Application
 * Length, Weight, Temperature, Speed conversions
 */

#ifndef CONVERTER_APP_H
#define CONVERTER_APP_H

#include <Arduino.h>
#include "config.h"

enum ConverterCategory {
  CONVERT_LENGTH = 0,
  CONVERT_WEIGHT,
  CONVERT_TEMPERATURE,
  CONVERT_SPEED,
  CONVERT_CATEGORY_COUNT
};

struct ConversionUnit {
  const char* name;
  const char* symbol;
  float to_base;  // Conversion factor to base unit
};

void initConverterApp();
void drawConverterApp();
void handleConverterTouch(TouchGesture& gesture);
float convertValue(float value, int fromUnit, int toUnit, ConverterCategory category);

#endif
