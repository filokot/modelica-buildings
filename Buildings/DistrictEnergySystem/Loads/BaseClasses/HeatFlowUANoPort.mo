within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model HeatFlowUANoPort
  "Model for computing the heat flow rate (from liquid to the load)"
  extends Modelica.Blocks.Icons.Block;

  parameter Real n(min=0.5, max=1.5) = 1.3
   "Exponent for heat transfer: Q_flow = UA * DeltaT^n";
  parameter Boolean homotopyInitialization = true "= true, use homotopy method"
    annotation(Evaluate=true, Dialog(tab="Advanced"));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput UA(
   quantity="ThermalConductance",
   unit="W/K",
   displayUnit="W/K",
   min=0)
    "Thermal conductance"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TInd(
    quantity="ThermodynamicTemperature",
    unit="K",
    displayUnit="degC") "Building indoor temperature"
    annotation (Placement(transformation(extent={{-140,50},{-100,90}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput Q_flow(
    quantity="HeatFlowRate",
    unit="W",
    displayUnit="W") "Heat flow rate at current conditions"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TLiq(
    quantity="ThermodynamicTemperature",
    unit="K",
    displayUnit="degC") "Liquid temperature" annotation (Placement(
        transformation(extent={{-140,-90},{-100,-50}})));

   Modelica.SIunits.TemperatureDifference dT;
equation
  Q_flow = UA * abs(dT)^n * sign(dT);
  dT = TLiq - TInd;
  annotation (
        defaultComponentName="heaFloUA");
end HeatFlowUANoPort;
