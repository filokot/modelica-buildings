within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model HeatingOrCooling_LMTD
  extends Buildings.Fluid.Interfaces.LumpedVolumeDeclarations;
  extends Buildings.Fluid.Interfaces.TwoPortFlowResistanceParameters(dp_nominal=0);
  extends Buildings.Fluid.Interfaces.PartialTwoPortInterface(
    show_T=true,
    final m_flow_nominal=abs(Q_flow_nominal/cp_nominal/(T_a_nominal - T_b_nominal)));

  parameter Modelica.SIunits.Time tau = 30
    "Time constant at nominal flow (if energyDynamics <> SteadyState)"
     annotation (Dialog(tab = "Dynamics", group="Nominal condition"));

  // Advanced
  parameter Boolean homotopyInitialization = true "= true, use homotopy method"
    annotation(Evaluate=true, Dialog(tab="Advanced"));

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

  Buildings.Fluid.Sensors.TemperatureTwoPort senTSup(
    redeclare package Medium=Medium,
    m_flow_nominal=m_flow_nominal,
    allowFlowReversal=allowFlowReversal) "Temperature sensor"
    annotation (Placement(transformation(extent={{-20,-11},{0,11}})));
  Buildings.Fluid.FixedResistances.PressureDrop res(
    redeclare final package Medium = Medium,
    final from_dp=from_dp,
    final show_T=show_T,
    final m_flow_nominal=m_flow_nominal,
    final allowFlowReversal=allowFlowReversal,
    final linearized=linearizeFlowResistance,
    final homotopyInitialization=homotopyInitialization,
    deltaM=deltaM,
    dp_nominal=dp_nominal) "Flow resistance"
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  Buildings.Fluid.MixingVolumes.MixingVolume vol(
    nPorts=2,
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
    final C_start=C_start) "Volume of fluid"
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={20,-10})));

  Real frac_Q_flow "Positive fractional heat flow rate (for development only)";
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b heaPorLoa
    "Heat port connected to the load" annotation (Placement(transformation(
          extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,
            110}})));
  HeatFlowUA_LMTD heaFloUA
    annotation (Placement(transformation(extent={{60,38},{80,58}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput y
    "Control signal for heating or cooling"
    annotation (Placement(transformation(extent={{-140,50},{-100,90}}),
        iconTransformation(extent={{-140,60},{-100,100}})));
  Modelica.Blocks.Sources.RealExpression offHvs(y=-y_small)
    "Offset control signal for smooth Heaviside function"
    annotation (Placement(transformation(extent={{-100,14},{-80,34}})));
  Buildings.Controls.OBC.CDL.Continuous.MultiSum mulSum(nin=2)
    annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
  Buildings.Utilities.Math.SmoothHeaviside smoothHeaviside(delta=
        y_small)
    annotation (Placement(transformation(extent={{-20,40},{0,60}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain           actUA(k=UA_nominal)
    "Computes the actual value of UA"
    annotation (Placement(transformation(extent={{20,40},{40,60}})));
protected
  parameter Modelica.SIunits.SpecificHeatCapacity cp_nominal=
    Medium.specificHeatCapacityCp(
      Medium.setState_pTX(Medium.p_default, T_a_nominal, Medium.X_default))
    "Specific heat capacity at nominal conditions";
  parameter Modelica.SIunits.ThermalConductance UA_nominal=
    Q_flow_nominal / abs(
    Buildings.DistrictEnergySystem.Loads.BaseClasses.logMeanTempDif(
    T1_a=T_a_nominal, T1_b=T_b_nominal, T2_a=TInd_nominal, T2_b=TInd_nominal))^n
    "Thermal conductance at nominal conditions";

  parameter Medium.ThermodynamicState sta_default=Medium.setState_pTX(
      T=Medium.T_default, p=Medium.p_default, X=Medium.X_default);
  parameter Modelica.SIunits.Density rho_default=Medium.density(sta_default)
    "Density, used to compute fluid volume";
  parameter Medium.ThermodynamicState sta_start=Medium.setState_pTX(
      T=T_start, p=p_start, X=X_start);
  parameter Modelica.SIunits.SpecificEnthalpy h_outflow_start = Medium.specificEnthalpy(sta_start)
    "Start value for outflowing enthalpy";
  parameter Real y_small = 1E-2;
equation
  frac_Q_flow = abs(heaPorLoa.Q_flow) / Q_flow_nominal;
  connect(port_a, res.port_a)
    annotation (Line(points={{-100,0},{-60,0}}, color={0,127,255}));
  connect(res.port_b, senTSup.port_a)
    annotation (Line(points={{-40,0},{-20,0}}, color={0,127,255}));
  connect(senTSup.port_b, vol.ports[1])
    annotation (Line(points={{0,0},{18,0}},       color={0,127,255}));
  connect(vol.ports[2], port_b)
    annotation (Line(points={{22,0},{100,0}}, color={0,127,255}));
  connect(y, mulSum.u[1]) annotation (Line(points={{-120,70},{-92,70},{-92,51},{
          -62,51}}, color={0,0,127}));
  connect(offHvs.y, mulSum.u[2]) annotation (Line(points={{-79,24},{-70,24},{-70,
          49},{-62,49}}, color={0,0,127}));
  connect(mulSum.y, smoothHeaviside.u) annotation (Line(points={{-39,50},{-30.5,
          50},{-30.5,50},{-22,50}}, color={0,0,127}));
  connect(smoothHeaviside.y, actUA.u)
    annotation (Line(points={{1,50},{18,50}}, color={0,0,127}));
  connect(actUA.y, heaFloUA.UA) annotation (Line(points={{41,50},{48,50},{48,55},
          {59,55}}, color={0,0,127}));
  connect(senTSup.T, heaFloUA.TLiqSup) annotation (Line(points={{-10,12.1},{-10,
          20.1},{52,20.1},{52,41},{59,41}}, color={0,0,127}));
  connect(vol.heatPort, heaFloUA.port_a) annotation (Line(points={{10,-10},{10,-40},
          {50,-40},{50,48},{60,48}}, color={191,0,0}));
  connect(heaFloUA.port_b, heaPorLoa) annotation (Line(points={{80,48},{92,48},{
          92,100},{0,100}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-100},{100,100}},
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
        coordinateSystem(preserveAspectRatio=false)));
end HeatingOrCooling_LMTD;
