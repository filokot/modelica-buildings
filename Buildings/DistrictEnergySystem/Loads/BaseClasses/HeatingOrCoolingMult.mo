within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model HeatingOrCoolingMult "Model for static heat transfer between a circulating fluid and a thermal load"
  extends Buildings.Fluid.Interfaces.LumpedVolumeDeclarations;
  extends Buildings.Fluid.Interfaces.PartialTwoPortInterface(
    show_T=true,
    final m_flow_nominal=sum(m_flowLoa_nominal));

  parameter Integer nLoa = 1
    "Number of connected loads";
  parameter Modelica.SIunits.Time tau = 30
    "Time constant at nominal flow (if energyDynamics <> SteadyState)"
     annotation (Dialog(tab = "Dynamics", group="Nominal condition"));
  // Advanced
  parameter Boolean homotopyInitialization = true "= true, use homotopy method"
    annotation(Evaluate=true, Dialog(tab="Advanced"));
  parameter Modelica.SIunits.PressureDifference dp_nominal=0
    "Pressure difference at nominal conditions"
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
  parameter Modelica.SIunits.HeatFlowRate Q_flowLoa_nominal[nLoa](
    each min=0)
    "Thermal power exchanged with each load at nominal conditions (>0)"
    annotation(Dialog(group = "Nominal condition"));
  parameter Modelica.SIunits.Temperature TLoa_nominal[nLoa](
    each min=Modelica.SIunits.Conversions.from_degC(0),
    each displayUnit="degC")
    "Representative temperature of the load at nominal conditions"
    annotation(Dialog(group = "Nominal condition"));
  parameter Boolean reverseAction = false
    "Set to true for throttling the water flow rate through a cooling coil controller";
  Buildings.Fluid.MixingVolumes.MixingVolume vol(
    redeclare package Medium=Medium,
    final m_flow_nominal=m_flow_nominal,
    final V=m_flow_nominal*tau/rho_default,
    final mSenFac=1,
    final allowFlowReversal=allowFlowReversal,
    final massDynamics=massDynamics,
    final energyDynamics=energyDynamics,
    final T_start=T_start,
    final p_start=p_start,
    final X_start=X_start,
    final C_start=C_start,
    final use_C_flow=false,
    final prescribedHeatFlowRate=true,
    final nPorts=2)
    "Volume of fluid"
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={50,10})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b heaPorLoa[nLoa]
    "Heat port transfering heat to the load"
    annotation (Placement(transformation(
          extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,
            110}})));
  HeatFlowEffectivenessMult heaFloEps(
   redeclare package Medium=Medium,
   final nLoa=nLoa,
   final m_flow_nominal=m_flow_nominal,
   final homotopyInitialization=homotopyInitialization,
   final dp_nominal=dp_nominal) "Compute the heat transfer with each load"
   annotation (Placement(transformation(extent={{-10,10},{10,-10}})));
  Modelica.Blocks.Sources.RealExpression UALoa[nLoa](y=UALoa_nominal) "Thermal conductance"
    annotation (Placement(transformation(extent={{-100,-46},{-80,-26}})));
  final parameter Modelica.SIunits.ThermalConductance UA_nominal = sum(
    UALoa_nominal .* m_flowLoa_nominal) / m_flow_nominal
     "Thermal conductance at nominal conditions";
  final parameter Modelica.SIunits.HeatFlowRate Q_flow_nominal = sum(Q_flowLoa_nominal)
     "Total thermal power exchanged with the loads at nominal conditions (>0)";
  final parameter Modelica.SIunits.MassFlowRate m_flowLoa_nominal[nLoa] = abs(
    Q_flowLoa_nominal / cp_nominal / (T_a_nominal - T_b_nominal))
    "Water mass flow rate at nominal conditions";
  final parameter Modelica.SIunits.ThermalConductance UALoa_nominal[nLoa]=
    Q_flowLoa_nominal ./ abs(Buildings.DistrictEnergySystem.Loads.BaseClasses.logMeanTempDif(
        T1_a=T_a_nominal, T1_b=T_b_nominal, T2_a=TLoa_nominal, T2_b=TLoa_nominal))
    "Thermal conductance at nominal conditions";
  Buildings.DistrictEnergySystem.Loads.BaseClasses.EffectivenessControl effectivenessControl[nLoa](
    Q_flow_nominal=Q_flowLoa_nominal,
    m_flow_nominal=m_flowLoa_nominal,
    each reverseAction=reverseAction)
    annotation (Placement(transformation(extent={{-60,42},{-40,60}})));
  Modelica.Blocks.Sources.RealExpression TInlVal[nLoa](y=fill(heaFloEps.TInl, nLoa)) "Fluid inlet temperature"
    annotation (Placement(transformation(extent={{-100,54},{-80,74}})));
  Modelica.Blocks.Sources.RealExpression cpInlVal[nLoa](y=fill(heaFloEps.cpInl, nLoa))
    "Fluid inlet specific heat capacity" annotation (Placement(transformation(extent={{-100,38},{-80,58}})));
  Modelica.Blocks.Sources.RealExpression heaPorTVal[nLoa](y=heaPorLoa.T) "Temperature of the load"
    annotation (Placement(transformation(extent={{-100,22},{-80,42}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput Q_flowLoaReq[nLoa](quantity="HeatFlowRate")
    "Heat flow rate required to meet the load temperature setpoint" annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,90}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,80})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput m_flowReq(quantity="MassFlowRate")
    "Total mass flow rate to provide the required heat flow rates"
    annotation (Placement(transformation(extent={{100,70},{120,90}}), iconTransformation(extent={{100,70},{120,90}})));
  Buildings.Controls.OBC.CDL.Continuous.MultiSum mulSum(nin=nLoa) "Sum the mass flow rates of all loads"
    annotation (Placement(transformation(extent={{60,70},{80,90}})));

  // FOR DEVELOPMENT ONLY
  Real frac_Q_flow "Positive fractional heat flow rate";
  // FOR DEVELOPMENT ONLY

protected
  parameter Modelica.SIunits.SpecificHeatCapacity cp_nominal=
    Medium.specificHeatCapacityCp(
      Medium.setState_pTX(Medium.p_default, T_a_nominal, Medium.X_default))
    "Specific heat capacity at nominal conditions";

  parameter Medium.ThermodynamicState sta_default = Medium.setState_pTX(
      T=Medium.T_default, p=Medium.p_default, X=Medium.X_default);
  parameter Modelica.SIunits.Density rho_default = Medium.density(sta_default)
    "Density, used to compute fluid volume";
  parameter Medium.ThermodynamicState sta_start = Medium.setState_pTX(
      T=T_start, p=p_start, X=X_start);
  parameter Modelica.SIunits.SpecificEnthalpy h_outflow_start = Medium.specificEnthalpy(sta_start)
    "Start value for outflowing enthalpy";
equation

  // FOR DEVELOPMENT ONLY
  frac_Q_flow = abs(heaFloEps.Q_flow / Q_flow_nominal);
  // FOR DEVELOPMENT ONLY

  connect(heaFloEps.port_b, vol.ports[1]) annotation (Line(points={{10,0},{52,0}},  color={0,127,255}));
  connect(vol.ports[2], port_b) annotation (Line(points={{48,0},{100,0}}, color={0,127,255}));
  connect(heaFloEps.UA, UALoa.y) annotation (Line(points={{-12,-8},{-20,-8},{-20,-36},{-79,-36}}, color={0,0,127}));
  connect(port_a, heaFloEps.port_a) annotation (Line(points={{-100,0},{-10,0}}, color={0,127,255}));
  connect(heaFloEps.heaPor, heaPorLoa) annotation (Line(points={{0,8},{0,100}}, color={191,0,0}));
  connect(effectivenessControl.m_flow, heaFloEps.m_flowLoa)
    annotation (Line(points={{-39,55},{-20,55},{-20,8},{-12,8}}, color={0,0,127}));
  connect(UALoa.y, effectivenessControl.UA)
    annotation (Line(points={{-79,-36},{-70,-36},{-70,58},{-62,58}}, color={0,0,127}));
  connect(TInlVal.y, effectivenessControl.TInl)
    annotation (Line(points={{-79,64},{-68,64},{-68,50},{-62,50}}, color={0,0,127}));
  connect(Q_flowLoaReq, effectivenessControl.Q_flowReq)
    annotation (Line(points={{-120,90},{-66,90},{-66,54},{-62,54}}, color={0,0,127}));
  connect(m_flowReq, mulSum.y) annotation (Line(points={{110,80},{81,80}}, color={0,0,127}));
  connect(effectivenessControl.m_flow, mulSum.u)
    annotation (Line(points={{-39,55},{40,55},{40,80},{58,80}}, color={0,0,127}));
  connect(heaPorTVal.y, effectivenessControl.TLoad)
    annotation (Line(points={{-79,32},{-66,32},{-66,42},{-62,42}}, color={0,0,127}));
  connect(cpInlVal.y, effectivenessControl.cpInl)
    annotation (Line(points={{-79,48},{-72,48},{-72,46},{-62,46}}, color={0,0,127}));
  annotation (defaultComponentName="heaOrCoo",
 Documentation(info="<html>
 <p>
 The heat flow rate between the fluid and the load is computed based on an
 exponential relationship to a representative temperature difference cf.
 <a href=\"modelica://Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatFlowUA_LMTD\">
 Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatFlowUA_LMTD</a>. Using the logarithmic
 mean temperature difference, <code>dTLog = (TFluid_a - TFluid_b) / ln((TFluid_a - TLoad) /
 (TFluid_b - TLoad))</code> results from integrating the local equation of the heat flow rate
 over the heat transfer area.
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
  Icon(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-60},{100,100}},
        grid={1,1}), graphics={
        Rectangle(
          extent={{-70,70},{70,-70}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-101,5},{100,-4}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{0,-4},{100,5}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={255,0,0},
          fillPattern=FillPattern.Solid),
          Rectangle(extent={{-100,100},{100,-100}}, lineColor={95,95,95})}),
                                            Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-60},{100,100}})));
end HeatingOrCoolingMult;
