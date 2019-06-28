within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model FirstOrderODE "Simplified first order ODE model for computing indoor temperature"
  extends Modelica.Blocks.Icons.Block;
  parameter Modelica.SIunits.Temperature TOutHea_nominal(displayUnit="degC")
    "Outdoor temperature at heating nominal conditions"
    annotation(Dialog(group = "Nominal condition"));
  parameter Modelica.SIunits.Temperature TIndHea_nominal(displayUnit="degC")
    "Indoor temperature at heating nominal conditions"
    annotation(Dialog(group = "Nominal condition"));
  parameter Modelica.SIunits.HeatFlowRate Q_flowHea_nominal
    "Heating (>0) heat flow rate at nominal conditions"
    annotation(Dialog(group = "Nominal condition"));
  parameter Modelica.SIunits.HeatFlowRate Q_flow_nominal = Q_flowHea_nominal
    "Heating (>0) or cooling (<0) heat flow rate at nominal conditions"
    annotation(Dialog(group = "Nominal condition"));
  parameter Boolean steadyStateInitial = false
    "true initializes T with dT(0)/dt=0, false initializes T with T(0)=TIndHea_nominal"
     annotation (Dialog(group="Initialization"), Evaluate=true);
  parameter Modelica.SIunits.Time tau = 7200
    "Time constant for room air temperature dynamics";
  Buildings.Controls.OBC.CDL.Interfaces.RealInput TOut(
    quantity="ThermodynamicTemperature", unit="K", displayUnit="degC")
    "Outdoor temperature"
    annotation (Placement(transformation(extent={{-140,70},{-100,110}}), iconTransformation(extent={{-140,60},{-100,100}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput TSet(
    quantity="ThermodynamicTemperature", unit="K", displayUnit="degC")
    "Setpoint temperature for heating or cooling"
    annotation (Placement(transformation(extent={{-140,
    10},{-100,50}}), iconTransformation(extent={{-140,8},{-100,48}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput Q_flowReq(
    quantity="HeatFlowRate", unit="W")
    "Required heat flow rate to meet setpoint temperature"
    annotation (Placement(transformation(extent={{-140,
    -50},{-100,-10}}), iconTransformation(extent={{-140,-50},{-100,-10}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput Q_flowAct(
    quantity="HeatFlowRate", unit="W")
    "Actual heating or cooling heat flow rate"
    annotation (Placement(transformation(extent={{-140,-110},{-100,
    -70}}), iconTransformation(extent={{-140,-100},{-100,-60}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput TInd(
    quantity="ThermodynamicTemperature", unit="K", displayUnit="degC") "Indoor temperature"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
protected
  parameter Modelica.SIunits.ThermalConductance G = -Q_flowHea_nominal / (TOutHea_nominal - TIndHea_nominal)
  "Lumped thermal conductance representing all deltaT dependent heat transfer mechanisms";
initial equation
  if steadyStateInitial then
    der(TInd) = 0;
  else
    TInd = TIndHea_nominal;
  end if;
equation
  der(TInd) * tau = (Q_flowAct - Q_flowReq) / G + TSet - TInd;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
  Diagram(coordinateSystem(preserveAspectRatio=false)));
end FirstOrderODE;
