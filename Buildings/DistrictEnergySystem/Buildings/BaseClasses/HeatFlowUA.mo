within Buildings.DistrictEnergySystem.Buildings.BaseClasses;
model HeatFlowUA
  "Model for computing the heat flow rate based on UA input value"
  extends Modelica.Blocks.Icons.Block;

  parameter Real n(min=0.5, max=1.5)
   "Exponent for heat transfer: Q_flow = UA * DeltaT^n";

  Modelica.Blocks.Interfaces.RealInput UA(
   quantity="ThermalConductance",
   unit="W/K",
   displayUnit="W/K",
   min=0)
    "Thermal conductance"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));

  Modelica.Blocks.Interfaces.RealInput TInd(
    quantity="ThermodynamicTemperature",
    unit="K",
    displayUnit="degC") "Building indoor temperature"
    annotation (Placement(transformation(extent={{-140,60},{-100,100}})));

  Modelica.Blocks.Interfaces.RealInput TLiq(
    quantity="ThermodynamicTemperature",
    unit="K",
    displayUnit="degC") "Liquid temperature"
    annotation (Placement(transformation(extent={{-140,-100},{-100,-60}})));

  Modelica.Blocks.Interfaces.RealOutput Q_flow(
    each quantity="HeatFlowRate",
    each unit="W",
    each displayUnit="W")
    "Heat flow rate at current conditions"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));

equation
  Q_flow = UA * abs(TLiq - TInd)^n * sign(TLiq - TInd);

  annotation (
        defaultComponentName="heaFloUA");
end HeatFlowUA;
