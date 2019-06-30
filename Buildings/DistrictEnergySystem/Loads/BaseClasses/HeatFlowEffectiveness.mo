within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model HeatFlowEffectiveness
  "Model for static heat transfer between a circulating fluid and a thermal load, based on effectiveness"
  extends Buildings.Fluid.Interfaces.StaticTwoPortHeatMassExchanger(
    final sensibleOnly=true,
    final prescribedHeatFlowRate=false,
    dp_nominal=0,
    Q_flow=sum(heaPor.Q_flow),
    mWat_flow=0,
    heaInp(y=Q_flow));
  parameter Integer nLoa
    "Number of loads";
  parameter Real expUA = 4/5
    "Exponent of Reynolds number in the flow correlation used for computing UA internal value";
  parameter Modelica.SIunits.ThermalConductance UALoaExt_nominal[nLoa]
    "External thermal conductance at nominal conditions";
  parameter Modelica.SIunits.ThermalConductance UALoaInt_nominal[nLoa]
    "Internal thermal conductance at nominal conditions";
  parameter Modelica.SIunits.MassFlowRate m_flowLoa_nominal[nLoa]
    "Water mass flow rate at nominal conditions";
  Modelica.SIunits.Temperature port_a_T
  "Temperature of ideally mixed fluid at component side of port_a";
  Modelica.SIunits.Temperature port_b_T
  "Temperature of ideally mixed fluid at component side of port_b";
  Modelica.SIunits.Temperature TInl
  "Temperature of ideally mixed fluid at component side of inlet port";
  Modelica.SIunits.SpecificHeatCapacity cpInl
  "Specific heat capacity of ideally mixed fluid at component side of inlet port";
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heaPor[nLoa]
    annotation (Placement(transformation(extent={{-10,-90},{10,-70}}),
                iconTransformation(extent={{-10,-90},{10,-70}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput m_flowLoa[nLoa](
    each quantity="MassFlowRate") "Mass flow rate" annotation (
      Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,80}),  iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,-80})));
  Buildings.DistrictEnergySystem.Loads.BaseClasses.EffectivenessDirect effDir[nLoa]
    annotation (Placement(transformation(extent={{-20,78},{0,98}})));
  Modelica.Blocks.Sources.RealExpression TInlVal[nLoa](y=fill(TInl, nLoa))
    "Fluid inlet temperature"
    annotation (Placement(transformation(extent={{-80,58},{-60,78}})));
  Modelica.Blocks.Sources.RealExpression cpInlVal[nLoa](y=fill(cpInl, nLoa))
    "Fluid inlet specific heat capacity"
    annotation (Placement(transformation(extent={{-80,44},{-60,64}})));
  Modelica.Blocks.Sources.RealExpression heaPorTVal[nLoa](y=heaPor.T)
    "Temperature of the load"
    annotation (Placement(transformation(extent={{-80,30},{-60,50}})));
  Modelica.Blocks.Sources.RealExpression m_flowActVal[nLoa](y=m_flowAct) "Actual mass flow rate"
    annotation (Placement(transformation(extent={{-80,72},{-60,92}})));
  // Parameters for inverseXRegularized.
  // These are assigned here for efficiency reason.
  // Otherwise, they would need to be computed each time
  // the function is invocated.
  Controls.OBC.CDL.Interfaces.RealOutput UAAct[nLoa](quantity="ThermalConductance")
    "Thermal conductance at actual flow rate"
    annotation (Placement(transformation(extent={{100,70},{120,90}}), iconTransformation(extent={{100,60},{120,80}})));
  Modelica.Blocks.Sources.RealExpression UAActVal[nLoa](y=UAAct) "Thermal conductance at actual flow rate"
    annotation (Placement(transformation(extent={{-80,86},{-60,106}})));
protected
  final parameter Real deltaReg = m_flow_small/1E3
    "Smoothing region for inverseXRegularized";
  final parameter Real deltaInvReg = 1/deltaReg
    "Inverse value of delta for inverseXRegularized";
  final parameter Real aReg = -15*deltaInvReg
    "Polynomial coefficient for inverseXRegularized";
  final parameter Real bReg = 119*deltaInvReg^2
    "Polynomial coefficient for inverseXRegularized";
  final parameter Real cReg = -361*deltaInvReg^3
    "Polynomial coefficient for inverseXRegularized";
  final parameter Real dReg = 534*deltaInvReg^4
    "Polynomial coefficient for inverseXRegularized";
  final parameter Real eReg = -380*deltaInvReg^5
    "Polynomial coefficient for inverseXRegularized";
  final parameter Real fReg = 104*deltaInvReg^6
    "Polynomial coefficient for inverseXRegularized";
  Modelica.SIunits.MassFlowRate m_flowAct[nLoa];
equation
  UAAct = 1 ./ (1 ./ (UALoaInt_nominal .*
     Buildings.Utilities.Math.Functions.regNonZeroPower(m_flowAct ./ m_flowLoa_nominal, expUA)) +
     1 ./ UALoaExt_nominal);
  m_flowAct = Modelica.Fluid.Utilities.regStep(
    sum(m_flowLoa) - 2 * m_flow_small,
    m_flowLoa / sum(m_flowLoa) * m_flow,
    fill(0, nLoa),
    m_flow_small);
  heaPor.Q_flow = effDir.Q_flow;
  if allowFlowReversal then
    TInl = Modelica.Fluid.Utilities.regStep(
      x=m_flow,
      y1=port_a_T,
      y2=port_b_T,
      x_small=m_flow_small);
  else
    TInl = port_a_T;
  end if;
  port_a_T = Medium.temperature(
    Medium.setState_phX(
      p=port_a.p,
      h=inStream(port_a.h_outflow),
      X=inStream(port_a.Xi_outflow)));
  port_b_T = Medium.temperature(
    Medium.setState_phX(
      p=port_b.p,
      h=inStream(port_b.h_outflow),
      X=inStream(port_b.Xi_outflow)));
  cpInl = Medium.specificHeatCapacityCp(
    Medium.setState_pTX(
      p=port_a.p,
      T=TInl));
  connect(heaPorTVal.y, effDir.TLoad) annotation (Line(points={{-59,40},{-50,40},{-50,80},{-22,80}}, color={0,0,127}));
  connect(cpInlVal.y, effDir.cpInl) annotation (Line(points={{-59,54},{-52,54},{-52,84},{-22,84}}, color={0,0,127}));
  connect(TInlVal.y, effDir.TInl) annotation (Line(points={{-59,68},{-54,68},{-54,88},{-22,88}}, color={0,0,127}));
  connect(m_flowActVal.y, effDir.m_flow)
    annotation (Line(points={{-59,82},{-56,82},{-56,92},{-22,92}}, color={0,0,127}));
  connect(UAActVal.y, effDir.UA) annotation (Line(points={{-59,96},{-22,96}}, color={0,0,127}));
  annotation (
  defaultComponentName="heaFloEps",
  Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,80},{100,-80}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillColor={192,192,192},
          fillPattern=FillPattern.Forward)}),                    Diagram(coordinateSystem(preserveAspectRatio=false)));
end HeatFlowEffectiveness;
