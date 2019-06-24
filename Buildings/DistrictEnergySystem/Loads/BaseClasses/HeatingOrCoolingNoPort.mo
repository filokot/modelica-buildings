within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model HeatingOrCoolingNoPort
  "Model for coupling a building model and an substation model"
  extends Buildings.Fluid.Interfaces.TwoPortHeatMassExchanger(
    redeclare final Buildings.Fluid.MixingVolumes.MixingVolume vol(nPorts=3),
    T_start=TInd_nominal,
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
  parameter Modelica.SIunits.Temperature TInd_nominal(
    min=Modelica.SIunits.Conversions.from_degC(0),
    displayUnit="degC")
    "Building indoor temperature at nominal conditions"
    annotation(Dialog(group = "Nominal condition"));
  parameter Real n = 1.3 "Exponent for heat transfer: Q_flow = UA * DeltaT^n";

  HeatFlowUANoPort heaFloUA(n=n)
    annotation (Placement(transformation(extent={{20,60},{40,80}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput Q_flowAva(
    final min=0,
    quantity="HeatFlowRate",
    unit="W",
    displayUnit="W") "Unmet heating load"
    annotation (Placement(transformation(extent={{100,40},{120,60}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput           TInd(
    quantity="ThermodynamicTemperature",
    unit="K",
    displayUnit="degC") "Building indoor temperature"
    annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
  Buildings.Fluid.Sensors.Temperature senTRet(redeclare package Medium = Medium)
    "Liquid return temperature"
    annotation (Placement(transformation(extent={{-10,20},{10,40}})));
  Modelica.Blocks.Sources.RealExpression UAValue(y=UA_nominal)
    annotation (Placement(transformation(extent={{-100,54},{-80,74}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput Q_flowAct(
    quantity="HeatFlowRate",
    unit="W",
    displayUnit="W") annotation (Placement(transformation(extent={{-140,-100},{-100,
            -60}})));
  Buildings.HeatTransfer.Sources.PrescribedHeatFlow Q_flowToVol
    "Heat flow rate into the liquid volume"
    annotation (Placement(transformation(extent={{-58,-90},{-38,-70}})));
  Buildings.Controls.OBC.CDL.Continuous.MultiSum mulSum(nin=1, k={-1})
    annotation (Placement(transformation(extent={{-88,-90},{-68,-70}})));
  Real frac_Q_flow "Positive fractional heat flow rate (for development only)";
protected
  parameter Modelica.SIunits.SpecificHeatCapacity cp_nominal=
    Medium.specificHeatCapacityCp(
      Medium.setState_pTX(Medium.p_default, T_a_nominal, Medium.X_default))
    "Specific heat capacity at nominal conditions";
  parameter Modelica.SIunits.ThermalConductance UA_nominal=
    Q_flow_nominal / abs(T_b_nominal - TInd_nominal)^n
    "Thermal conductance at nominal conditions";
initial equation
  assert(abs(T_a_nominal - T_b_nominal) > 0,
    "In BuildingCouplingETS, T_a_nominal and T_b_nominal must have different values.");
  if T_a_nominal > T_b_nominal then
    assert(T_b_nominal > TInd_nominal,
      "In BuildingCouplingETS, T_a_nominal > T_b_nominal implies heating. At nominal conditions,
      the return water temperature must be strictly higher than the building indoor temperature.");
  else
    assert(T_b_nominal < TInd_nominal,
      "In BuildingCouplingETS, T_a_nominal < T_b_nominal implies cooling. At nominal conditions,
      the return water temperature must be strictly lower than the building indoor temperature.");
  end if;
equation
  frac_Q_flow = abs(Q_flowAct) / Q_flow_nominal;
  connect(heaFloUA.Q_flow, Q_flowAva) annotation (Line(points={{41,70},{56,70},
          {56,50},{110,50}},color={0,0,127}));
  connect(TInd, heaFloUA.TInd)
    annotation (Line(points={{-120,80},{-50,80},{-50,77},{18,77}},
                                                 color={0,0,127}));
  connect(vol.ports[3], senTRet.port)
    annotation (Line(points={{1,0},{0,0},{0,20}}, color={0,127,255}));
  connect(UAValue.y, heaFloUA.UA)
    annotation (Line(points={{-79,64},{-12,64},{-12,70},{18,70}},
                                                color={0,0,127}));
  connect(Q_flowToVol.port, vol.heatPort) annotation (Line(points={{-38,-80},{-20,
          -80},{-20,-10},{-9,-10}}, color={191,0,0}));
  connect(Q_flowAct, mulSum.u[1])
    annotation (Line(points={{-120,-80},{-90,-80}}, color={0,0,127}));
  connect(Q_flowToVol.Q_flow, mulSum.y)
    annotation (Line(points={{-58,-80},{-67,-80}}, color={0,0,127}));
  connect(senTRet.T, heaFloUA.TLiq) annotation (Line(points={{7,30},{12,30},{12,
          63},{18,63}}, color={0,0,127}));
    annotation (defaultComponentName="heaOrCoo",
              Icon(coordinateSystem(preserveAspectRatio=true), graphics={
          Rectangle(extent={{-100,100},{100,-100}}, lineColor={95,95,95})}),
                                                                Diagram(
        coordinateSystem(preserveAspectRatio=true)));
end HeatingOrCoolingNoPort;
