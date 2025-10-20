#include "globals.h"
#include "secondaryTables.h"
#include "corrections.h"
#include "maths.h"
#include "sensors.h"

void calculateSecondaryFuel(void)
{
  static bool aeActive = false;

  static int16_t triggeredTPSDOT = 0;
  int16_t TPS_change = 0;
  const uint32_t now = micros_safe();

  // Calculate TPSdot START
  // TPS_change = current.tpsDOT; // For testing
  TPS_change = (currentStatus.TPS - currentStatus.TPSlast);
  currentStatus.tpsDOT = (TPS_READ_FREQUENCY * TPS_change) / 2; // This is the % per second that the TPS has moved, adjusted for the 0.5% resolution of the TPS
  // Calculate TPSdot END

  const uint16_t tpsdotBin = (uint16_t)currentStatus.tpsDOT / 8; // This gets the TPS Dot BIN (0-255) for the TPSdot value
  // const uint16_t tpsdotBin = (uint16_t)getLoad(LOAD_SOURCE_TPSDOT, currentStatus); // This gets the TPS Dot BIN (0-255) for the TPSdot value
  uint16_t timeBin = get3DTableValue(&afrTable, tpsdotBin, (table3d_axis_t)currentStatus.RPM);
  uint16_t timeDelayMs = (uint16_t)(timeBin * 392UL / 100); // same as *3.92

  Serial.print("TPSChange: ");
  Serial.print(TPS_change);
  Serial.print(" TPSdot: ");
  Serial.print(currentStatus.tpsDOT);


  if (!aeActive && (currentStatus.tpsDOT > configPage2.taeThresh) && (abs(TPS_change) >= configPage2.taeMinChange)) // If ae not active set it up.
  {
    triggeredTPSDOT = currentStatus.tpsDOT;
    // Apply the AE if the TPSdot exceeds the threshold
    // Get the amount of time required
    aeActive = true; // Set AE as on
    currentStatus.AEEndTime = now + (uint32_t)timeDelayMs * 1000UL;
    BIT_SET(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); // Set the bit indicating that the 2nd fuel table is in use.
    currentStatus.VE2 = getVE2();                             // Get the VE2 value and apply it
    auto combinedVE = percentage(currentStatus.VE2, currentStatus.VE1);
    currentStatus.VE = (uint8_t)min((uint32_t)UINT8_MAX, combinedVE);
  }
  else if (aeActive) // If the time has elapsed, turn off AE
  {
    // First check if the tpsdot has increased if so change the end time etc
    if (currentStatus.tpsDOT > triggeredTPSDOT)
    {
      triggeredTPSDOT = currentStatus.tpsDOT;
      // Recalc it all and change the end time.
      currentStatus.AEEndTime = now + (uint32_t)timeDelayMs * 1000UL;
      currentStatus.VE2 = getVE2(); // Get the VE2 value and apply it
      auto combinedVE = percentage(currentStatus.VE2, currentStatus.VE1);
      currentStatus.VE = (uint8_t)min((uint32_t)UINT8_MAX, combinedVE);
    }
    else
    { // Otherwise make sure the current ve isnt being overwritten
      auto combinedVE = percentage(currentStatus.VE2, currentStatus.VE1);
      currentStatus.VE = (uint8_t)min((uint32_t)UINT8_MAX, combinedVE);
    }

    if (now >= currentStatus.AEEndTime)
    {
      aeActive = false; // Set AE as off
      currentStatus.VE2 = 0;
      BIT_CLEAR(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); // Clear the bit indicating that the 2nd fuel table is in use.
    }
    // AE is still active, keep VE2 applied
  }
}

// void calculateSecondaryFuel(void)
// {
//   //If the secondary fuel table is in use, also get the VE value from there
//   BIT_CLEAR(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); //Clear the bit indicating that the 2nd fuel table is in use.
//   if(configPage10.fuel2Mode > 0)
//   {
//     if(configPage10.fuel2Mode == FUEL2_MODE_MULTIPLY)
//     {
//       currentStatus.VE2 = getVE2();
//       //Fuel 2 table is treated as a % value. Table 1 and 2 are multiplied together and divided by 100
//       uint16_t combinedVE = ((uint16_t)currentStatus.VE1 * (uint16_t)currentStatus.VE2) / 100;
//       if(combinedVE <= UINT8_MAX) { currentStatus.VE = combinedVE; }
//       else { currentStatus.VE = UINT8_MAX; }
//     }
//     else if(configPage10.fuel2Mode == FUEL2_MODE_ADD)
//     {
//       currentStatus.VE2 = getVE2();
//       //Fuel tables are added together, but a check is made to make sure this won't overflow the 8-bit VE value
//       uint16_t combinedVE = (uint16_t)currentStatus.VE1 + (uint16_t)currentStatus.VE2;
//       if(combinedVE <= UINT8_MAX) { currentStatus.VE = combinedVE; }
//       else { currentStatus.VE = UINT8_MAX; }
//     }
//     else if(configPage10.fuel2Mode == FUEL2_MODE_CONDITIONAL_SWITCH )
//     {
//       if(configPage10.fuel2SwitchVariable == FUEL2_CONDITION_RPM)
//       {
//         if(currentStatus.RPM > configPage10.fuel2SwitchValue)
//         {
//           BIT_SET(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); //Set the bit indicating that the 2nd fuel table is in use.
//           currentStatus.VE2 = getVE2();
//           currentStatus.VE = currentStatus.VE2;
//         }
//       }
//       else if(configPage10.fuel2SwitchVariable == FUEL2_CONDITION_MAP)
//       {
//         if(currentStatus.MAP > configPage10.fuel2SwitchValue)
//         {
//           BIT_SET(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); //Set the bit indicating that the 2nd fuel table is in use.
//           currentStatus.VE2 = getVE2();
//           currentStatus.VE = currentStatus.VE2;
//         }
//       }
//       else if(configPage10.fuel2SwitchVariable == FUEL2_CONDITION_TPS)
//       {
//         if(currentStatus.TPS > configPage10.fuel2SwitchValue)
//         {
//           BIT_SET(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); //Set the bit indicating that the 2nd fuel table is in use.
//           currentStatus.VE2 = getVE2();
//           currentStatus.VE = currentStatus.VE2;
//         }
//       }
//       else if(configPage10.fuel2SwitchVariable == FUEL2_CONDITION_ETH)
//       {
//         if(currentStatus.ethanolPct > configPage10.fuel2SwitchValue)
//         {
//           BIT_SET(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); //Set the bit indicating that the 2nd fuel table is in use.
//           currentStatus.VE2 = getVE2();
//           currentStatus.VE = currentStatus.VE2;
//         }
//       }
//     }
//     else if(configPage10.fuel2Mode == FUEL2_MODE_INPUT_SWITCH)
//     {
//       if(digitalRead(pinFuel2Input) == configPage10.fuel2InputPolarity)
//       {
//         BIT_SET(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); //Set the bit indicating that the 2nd fuel table is in use.
//         currentStatus.VE2 = getVE2();
//         currentStatus.VE = currentStatus.VE2;
//       }
//     }
//   }
// }

void calculateSecondarySpark(void)
{
  // Same as above but for the secondary ignition table
  BIT_CLEAR(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE); // Clear the bit indicating that the 2nd spark table is in use.
  if (configPage10.spark2Mode > 0)
  {
    if (configPage10.spark2Mode == SPARK2_MODE_MULTIPLY)
    {
      BIT_SET(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE);
      currentStatus.advance2 = getAdvance2();
      // make sure we don't have a negative value in the multiplier table (sharing a signed 8 bit table)
      if (currentStatus.advance2 < 0)
      {
        currentStatus.advance2 = 0;
      }
      // Spark 2 table is treated as a % value. Table 1 and 2 are multiplied together and divided by 100
      int16_t combinedAdvance = ((int16_t)currentStatus.advance1 * (int16_t)currentStatus.advance2) / 100;
      // make sure we don't overflow and accidentally set negative timing, currentStatus.advance can only hold a signed 8 bit value
      if (combinedAdvance <= 127)
      {
        currentStatus.advance = combinedAdvance;
      }
      else
      {
        currentStatus.advance = 127;
      }
    }
    else if (configPage10.spark2Mode == SPARK2_MODE_ADD)
    {
      BIT_SET(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE); // Set the bit indicating that the 2nd spark table is in use.
      currentStatus.advance2 = getAdvance2();
      // Spark tables are added together, but a check is made to make sure this won't overflow the 8-bit VE value
      int16_t combinedAdvance = (int16_t)currentStatus.advance1 + (int16_t)currentStatus.advance2;
      // make sure we don't overflow and accidentally set negative timing, currentStatus.advance can only hold a signed 8 bit value
      if (combinedAdvance <= 127)
      {
        currentStatus.advance = combinedAdvance;
      }
      else
      {
        currentStatus.advance = 127;
      }
    }
    else if (configPage10.spark2Mode == SPARK2_MODE_CONDITIONAL_SWITCH)
    {
      if (configPage10.spark2SwitchVariable == SPARK2_CONDITION_RPM)
      {
        if (currentStatus.RPM > configPage10.spark2SwitchValue)
        {
          BIT_SET(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE); // Set the bit indicating that the 2nd spark table is in use.
          currentStatus.advance2 = getAdvance2();
          currentStatus.advance = currentStatus.advance2;
        }
      }
      else if (configPage10.spark2SwitchVariable == SPARK2_CONDITION_MAP)
      {
        if (currentStatus.MAP > configPage10.spark2SwitchValue)
        {
          BIT_SET(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE); // Set the bit indicating that the 2nd spark table is in use.
          currentStatus.advance2 = getAdvance2();
          currentStatus.advance = currentStatus.advance2;
        }
      }
      else if (configPage10.spark2SwitchVariable == SPARK2_CONDITION_TPS)
      {
        if (currentStatus.TPS > configPage10.spark2SwitchValue)
        {
          BIT_SET(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE); // Set the bit indicating that the 2nd spark table is in use.
          currentStatus.advance2 = getAdvance2();
          currentStatus.advance = currentStatus.advance2;
        }
      }
      else if (configPage10.spark2SwitchVariable == SPARK2_CONDITION_ETH)
      {
        if (currentStatus.ethanolPct > configPage10.spark2SwitchValue)
        {
          BIT_SET(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE); // Set the bit indicating that the 2nd spark table is in use.
          currentStatus.advance2 = getAdvance2();
          currentStatus.advance = currentStatus.advance2;
        }
      }
    }
    else if (configPage10.spark2Mode == SPARK2_MODE_INPUT_SWITCH)
    {
      if (digitalRead(pinSpark2Input) == configPage10.spark2InputPolarity)
      {
        BIT_SET(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE); // Set the bit indicating that the 2nd spark table is in use.
        currentStatus.advance2 = getAdvance2();
        currentStatus.advance = currentStatus.advance2;
      }
    }

    // Apply the fixed timing correction manually. This has to be done again here if any of the above conditions are met to prevent any of the seconadary calculations applying instead of fixec timing
    currentStatus.advance = correctionFixedTiming(currentStatus.advance);
    currentStatus.advance = correctionCrankingFixedTiming(currentStatus.advance); // This overrides the regular fixed timing, must come last
  }
}

/**
 * @brief Looks up and returns the VE value from the secondary fuel table
 *
 * This performs largely the same operations as getVE() however the lookup is of the secondary fuel table and uses the secondary load source
 * @return byte
 */
byte getVE2(void)
{
  byte tempVE = 100;
  if (configPage10.fuel2Algorithm == LOAD_SOURCE_MAP)
  {
    // Speed Density
    currentStatus.fuelLoad2 = currentStatus.MAP;
  }
  else if (configPage10.fuel2Algorithm == LOAD_SOURCE_TPS)
  {
    // Alpha-N
    currentStatus.fuelLoad2 = currentStatus.TPS * 2;
  }
  else if (configPage10.fuel2Algorithm == LOAD_SOURCE_IMAPEMAP)
  {
    // IMAP / EMAP
    currentStatus.fuelLoad2 = ((int16_t)currentStatus.MAP * 100U) / currentStatus.EMAP;
  }
  else if (configPage10.fuel2Algorithm == LOAD_SOURCE_TPSDOT)
  {
    uint16_t dot = currentStatus.tpsDOT * 2;
    currentStatus.fuelLoad2 = (int16_t)(dot / 8);
    // return (int16_t)(dot / 8); // 1 bin = 8 %/s
  }

  else
  {
    currentStatus.fuelLoad2 = currentStatus.MAP;
  } // Fallback position
  tempVE = get3DTableValue(&fuelTable2, currentStatus.fuelLoad2, currentStatus.RPM); // Perform lookup into fuel map for RPM vs MAP value

  return tempVE;
}

/**
 * @brief Performs a lookup of the second ignition advance table. The values used to look this up will be RPM and whatever load source the user has configured
 *
 * @return byte The current target advance value in degrees
 */
byte getAdvance2(void)
{
  byte tempAdvance = 0;
  if (configPage10.spark2Algorithm == LOAD_SOURCE_MAP) // Check which fuelling algorithm is being used
  {
    // Speed Density
    currentStatus.ignLoad2 = currentStatus.MAP;
  }
  else if (configPage10.spark2Algorithm == LOAD_SOURCE_TPS)
  {
    // Alpha-N
    currentStatus.ignLoad2 = currentStatus.TPS * 2;
  }
  else if (configPage10.spark2Algorithm == LOAD_SOURCE_IMAPEMAP)
  {
    // IMAP / EMAP
    currentStatus.ignLoad2 = (currentStatus.MAP * 100) / currentStatus.EMAP;
  }
  else
  {
    currentStatus.ignLoad2 = currentStatus.MAP;
  }
  tempAdvance = get3DTableValue(&ignitionTable2, currentStatus.ignLoad2, currentStatus.RPM) - OFFSET_IGNITION; // As above, but for ignition advance

  // Perform the corrections calculation on the secondary advance value, only if it uses a switched mode
  if ((configPage10.spark2SwitchVariable == SPARK2_MODE_CONDITIONAL_SWITCH) || (configPage10.spark2SwitchVariable == SPARK2_MODE_INPUT_SWITCH))
  {
    tempAdvance = correctionsIgn(tempAdvance);
  }

  return tempAdvance;
}
