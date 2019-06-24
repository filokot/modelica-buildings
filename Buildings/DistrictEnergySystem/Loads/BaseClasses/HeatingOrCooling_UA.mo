within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model HeatingOrCooling_UA "Model for static heat transfer between a circulating fluid and a thermal load"
  extends Buildings.Fluid.Interfaces.TwoPortHeatMassExchanger(
    redeclare final Buildings.Fluid.MixingVolumes.MixingVolume vol,
    show_T=false,
    m_flow_nominal=abs(Q_flow_nominal/cp_nominal/(T_a_nominal - T_b_nominal)),
    dp_nominal=0);

  parameter Modelica.SIunits.HeatFlowRate Q_flow_nominal
    "Cooling or heating power at nominal conditions (always positive)"
    annotation(Dialog(group = "Nominal condition"));
  parameter Modelica.SIunits.Temperature T_a_nominal(
    min=Modelica.SIunits.Conversions.from_degC(0),
    displayUnit="degC")
    "Water supply temperature at nominal conditions"
    annotation(Dialog(group = "Nominal condition"));
  parameter Modelica.SIunits.Temperature T_b_nominal(
    min=Modelica.SIunits.Conversions.from_degC(0),
    displayUnit="degC")
    "Water return temperature at nominal conditions"
    annotation(Dialog(group = "Nominal condition"));
  parameter Modelica.SIunits.Temperature TLoa_nominal(
    min=Modelica.SIunits.Conversions.from_degC(0),
    displayUnit="degC")
    "Representative temperature of the load at nominal conditions"
    annotation(Dialog(group = "Nominal condition"));
  parameter Real n = 1.3 "Exponent for heat transfer: Q_flow = UA * DeltaT^n";

  Real frac_Q_flow "Positive fractional heat flow rate (for development only)";

  Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatFlowUA heaFloUA
    annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={70,20})));

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b heaPorLoa
    "Heat port connected to the load" annotation (Placement(transformation(
          extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,
            110}})));
  Buildings.Utilities.Math.SmoothHeaviside           smoothHeaviside(delta=y_small)
    annotation (Placement(transformation(extent={{-20,40},{0,60}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain actUA(k=UA_nominal)
    "Computes the actual value of UA"
    annotation (Placement(transformation(extent={{20,40},{40,60}})));
  Buildings.Controls.OBC.CDL.Continuous.MultiSum mulSum(nin=2)
    annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
  Modelica.Blocks.Sources.RealExpression offHvs(y=-y_small)
    "Offset control signal for smooth Heaviside function"
    annotation (Placement(transformation(extent={{-100,14},{-80,34}})));
  Controls.OBC.CDL.Interfaces.RealInput y
    "Control signal for heating or cooling"
    annotation (Placement(transformation(extent={{-140,60},{-100,100}}),
        iconTransformation(extent={{-140,60},{-100,100}})));
protected
  parameter Modelica.SIunits.SpecificHeatCapacity cp_nominal=
    Medium.specificHeatCapacityCp(
      Medium.setState_pTX(Medium.p_default, T_a_nominal, Medium.X_default))
    "Specific heat capacity at nominal conditions";
  parameter Modelica.SIunits.ThermalConductance UA_nominal=
    Q_flow_nominal / abs(T_b_nominal - TLoa_nominal)^n
    "Thermal conductance at nominal conditions";
  parameter Real y_small = 1E-2;
initial equation
  assert(abs(T_a_nominal - T_b_nominal) > 0,
    "In HeatingOrCooling, T_a_nominal and T_b_nominal must have different values.");
  if T_a_nominal > T_b_nominal then
    assert(T_b_nominal > TLoa_nominal,
      "In HeatingOrCooling, T_a_nominal > T_b_nominal implies heating. At nominal conditions,
      the return water temperature must be strictly higher than the temperature of the load.");
  else
    assert(T_b_nominal < TLoa_nominal,
      "In HeatingOrCooling, T_a_nominal < T_b_nominal implies cooling. At nominal conditions,
      the return water temperature must be strictly lower than the temperature of the load.");
  end if;
equation
  frac_Q_flow = abs(heaFloUA.Q_flow) / Q_flow_nominal;
  connect(vol.heatPort, heaFloUA.port_a) annotation (Line(points={{-9,-10},{-20,
          -10},{-20,20},{60,20}},           color={191,0,0}));
  connect(heaFloUA.port_b, heaPorLoa) annotation (Line(points={{80,20},{90,20},
          {90,80},{0,80},{0,100}}, color={191,0,0}));
  connect(actUA.y, heaFloUA.UA) annotation (Line(points={{41,50},{46,50},{46,27},
          {59,27}},      color={0,0,127}));
  connect(mulSum.y, smoothHeaviside.u)
    annotation (Line(points={{-39,50},{-22,50}}, color={0,0,127}));
  connect(actUA.u, smoothHeaviside.y)
    annotation (Line(points={{18,50},{1,50}}, color={0,0,127}));
  connect(y, mulSum.u[1]) annotation (Line(points={{-120,80},{-92,80},{-92,51},
          {-62,51}}, color={0,0,127}));
  connect(offHvs.y, mulSum.u[2]) annotation (Line(points={{-79,24},{-72,24},{
          -72,49},{-62,49}}, color={0,0,127}));
    annotation (defaultComponentName="heaOrCoo",
 Documentation(info="<html>
 <p>
 The heat flow rate between the fluid and the load is computed based on an 
 exponential relationship to a representative temperature difference cf.   
 <a href=\"modelica://Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatFlowUA\">
 Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatFlowUA</a>. Using the temperature of the mixing 
 volume (outlet temperature) ensures that the Second Law is respected i.e. that the fluid 
 temperature cannot cross the temperature of the load.
 </p>
 <p>
 The nominal UA-value (W/K) is calculated consistently from the given nominal cooling or 
 heating power, nominal fluid and load temperatures and the 
 exponent for heat transfer. The actual UA-value is equal to the nominal value 
 when there is a cooling or heating demand. It is equal to zero otherwise to prevent concomitant 
 cooling and heating when two instances of this class are connected to the same load model. The 
 model uses a smoothing function between those two conditions so that the actual 
 UA-value is continuously differentiable.
 </p>
 </html>"),
 Icon(coordinateSystem(preserveAspectRatio=true), graphics={
          Rectangle(extent={{-100,100},{100,-100}}, lineColor={95,95,95})}),
                                                                Diagram(
        coordinateSystem(preserveAspectRatio=true)));
end HeatingOrCooling_UA;
