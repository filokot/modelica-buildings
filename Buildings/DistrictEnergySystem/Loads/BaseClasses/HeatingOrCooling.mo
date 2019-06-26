within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model HeatingOrCooling "Model for static heat transfer between a circulating fluid and a thermal load"
  extends Buildings.Fluid.Interfaces.LumpedVolumeDeclarations;
  extends Buildings.Fluid.Interfaces.PartialTwoPortInterface(
    show_T=true,
    final m_flow_nominal=abs(Q_flow_nominal/cp_nominal/(T_a_nominal - T_b_nominal)));

  parameter Integer nLoa = 1
    "Number of connected loads";

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
  parameter Modelica.SIunits.Temperature TLoa_nominal(
    min=Modelica.SIunits.Conversions.from_degC(0),
    displayUnit="degC")
    "Representative temperature of the load at nominal conditions"
    annotation(Dialog(group = "Nominal condition"));
  parameter Modelica.SIunits.PressureDifference dp_nominal=0 "Pressure difference";
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
    nPorts=2)          "Volume of fluid"
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={30,10})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b heaPorLoa[nLoa]
    "Heat port transfering heat to the load"
    annotation (Placement(transformation(
          extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,
            110}})));
  Modelica.Blocks.Sources.RealExpression UA(y=UA_nominal) "Offset control signal for smooth Heaviside function"
    annotation (Placement(transformation(extent={{-100,30},{-80,50}})));
  Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatFlowEffectiveness heaFloEps(
   redeclare package Medium=Medium,
   final m_flow_nominal=m_flow_nominal,
   final homotopyInitialization=homotopyInitialization,
    dp_nominal=dp_nominal)
   annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));

  // FOR DEVELOPMENT ONLY
  Real frac_Q_flow "Positive fractional heat flow rate";

protected
  parameter Modelica.SIunits.SpecificHeatCapacity cp_nominal=
    Medium.specificHeatCapacityCp(
      Medium.setState_pTX(Medium.p_default, T_a_nominal, Medium.X_default))
    "Specific heat capacity at nominal conditions";
  parameter Modelica.SIunits.ThermalConductance UA_nominal=
    Q_flow_nominal / abs(
    Buildings.DistrictEnergySystem.Loads.BaseClasses.logMeanTempDif(
    T1_a=T_a_nominal, T1_b=T_b_nominal, T2_a=TLoa_nominal, T2_b=TLoa_nominal))
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
  // FOR DEVELOPMENT ONLY
  frac_Q_flow = abs(heaFloEps.heaPor.Q_flow / Q_flow_nominal);
  for i in 1:nLoa loop
    connect(heaFloEps.heaPor, heaPorLoa[i])
      annotation (Line(points={{-30,-8},{-30,-40},{80,-40},{80,100},{0,100}}, color={191,0,0}));
  end for;
  connect(port_a, heaFloEps.port_a) annotation (Line(points={{-100,0},{-40,0}}, color={0,127,255}));
  connect(UA.y, heaFloEps.UA) annotation (Line(points={{-79,40},{-60,40},{-60,8},{-42,8}}, color={0,0,127}));
  connect(heaFloEps.port_b, vol.ports[1]) annotation (Line(points={{-20,0},{32,0}}, color={0,127,255}));
  connect(vol.ports[2], port_b) annotation (Line(points={{28,0},{100,0}}, color={0,127,255}));
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
end HeatingOrCooling;
