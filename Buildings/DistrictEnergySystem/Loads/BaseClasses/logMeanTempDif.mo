within Buildings.DistrictEnergySystem.Loads.BaseClasses;
function logMeanTempDif "Logarithmic mean temperature difference"
  extends Modelica.Icons.Function;
  input Modelica.SIunits.ThermodynamicTemperature T1_a
  "Temperature of fluid 1 at port a";
  input Modelica.SIunits.ThermodynamicTemperature T1_b;
  input Modelica.SIunits.ThermodynamicTemperature T2_a;
  input Modelica.SIunits.ThermodynamicTemperature T2_b;
  output Modelica.SIunits.TemperatureDifference dT;
protected
  Modelica.SIunits.TemperatureDifference dT_a;
  Modelica.SIunits.TemperatureDifference dT_b;
algorithm
  dT_a := T1_a - T2_a;
  dT_b := T1_b - T2_b;
  dT := if dT_a * dT_b <= 0 then 0
     else if abs(dT_a - dT_b) > 5E-2 * max(abs(dT_a), abs(dT_b)) then
     (dT_a - dT_b) / log(dT_a / dT_b) else
     (dT_a + dT_b) / 2 * (1 - (dT_a - dT_b)^2 / (dT_a * dT_b) *
     (1 + (dT_a - dT_b)^2 / (dT_a * dT_b) / 2) / 12);
end logMeanTempDif;
