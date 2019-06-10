within Buildings.DistrictEnergySystem.Buildings.BaseClasses;
model HeatingOrCooling
  "Model for coupling a building model and an substation model"
  extends Fluid.Interfaces.TwoPortHeatMassExchanger(
    redeclare final Fluid.MixingVolumes.MixingVolume vol(nPorts=3),
    m_flow_nominal=abs(Q_flow_nominal/cp_nominal/(T_a_nominal - T_b_nominal)),
    tau=500,
    dp_nominal=0);

  parameter Modelica.SIunits.HeatFlowRate Q_flow_nominal
    "Cooling or heating heat flow rate at nominal conditions (always positive)"
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

  Fluid.Sensors.Temperature senTem(redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-10,26},{10,46}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort
   "Heatport for heat transfer with building indoor temperature"
   annotation (
      Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(
          extent={{-112,44},{-92,64}})));
  HeatFlowUA heaFloUA(n=n)
    annotation (Placement(transformation(extent={{40,50},{60,70}})));
  Modelica.Blocks.Sources.RealExpression heaPorT(y=heatPort.T)
    "Temperature at heat port"
    annotation (Placement(transformation(extent={{-60,70},{-40,90}})));
  Modelica.Blocks.Sources.RealExpression UAVal(y=UA_nominal) "UA value"
    annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
  HeatTransfer.Sources.PrescribedHeatFlow preHeaFloBui
    "Actual heat flow rate from ETS into building"
    annotation (Placement(transformation(extent={{72,50},{92,70}})));

  Controls.OBC.CDL.Continuous.MultiSum mulSum(nin=1, k={-1})
    annotation (Placement(transformation(extent={{54,-64},{34,-44}})));
  HeatTransfer.Sources.PrescribedHeatFlow preHeaFloETS
    "Actual heat flow rate from building into ETS"
    annotation (Placement(transformation(extent={{20,-64},{0,-44}})));

protected
  parameter Modelica.SIunits.SpecificHeatCapacity cp_nominal=
    Medium.specificHeatCapacityCp(
      Medium.setState_pTX(Medium.p_default, T_a_nominal, Medium.X_default))
    "Specific heat capacity at nominal conditions";
  parameter Modelica.SIunits.ThermalConductance UA_nominal=
    Q_flow_nominal / abs((T_a_nominal + T_b_nominal) / 2 - TInd_nominal)
    "Thermal conductance at nominal conditions";
initial equation
  assert(abs(T_a_nominal - T_b_nominal) > 0,
    "In BuildingCouplingETS, T_a_nominal and T_b_nominal must have different values.");
  if T_a_nominal > T_b_nominal then
    assert((T_a_nominal + T_b_nominal) / 2 > TInd_nominal,
      "In BuildingCouplingETS, T_a_nominal > T_b_nominal implies heating so at nominal conditions,
      the mean water temperature must be strictly higher than the building indoor temperature.");
  else
    assert((T_a_nominal + T_b_nominal) / 2 < TInd_nominal,
      "In BuildingCouplingETS, T_a_nominal < T_b_nominal implies cooling so at nominal conditions,
      the mean water temperature must be strictly lower than the building indoor temperature.");
  end if;
equation
  connect(senTem.T, heaFloUA.TLiq) annotation (Line(points={{7,36},{22,36},{22,52},
          {38,52}}, color={0,0,127}));
  connect(heaFloUA.TInd, heaPorT.y) annotation (Line(points={{38,68},{0,68},{0,80},
          {-39,80}}, color={0,0,127}));
  connect(UAVal.y, heaFloUA.UA)
    annotation (Line(points={{-39,60},{38,60}}, color={0,0,127}));
  connect(heaFloUA.Q_flow, preHeaFloBui.Q_flow)
    annotation (Line(points={{61,60},{72,60}}, color={0,0,127}));
  connect(preHeaFloBui.port, heatPort) annotation (Line(points={{92,60},{100,60},
          {100,100},{0,100}}, color={191,0,0}));
  connect(heaFloUA.Q_flow, mulSum.u[1]) annotation (Line(points={{61,60},{66,60},
          {66,-54},{56,-54}}, color={0,0,127}));
  connect(mulSum.y, preHeaFloETS.Q_flow)
    annotation (Line(points={{33,-54},{20,-54}}, color={0,0,127}));
  connect(preHeaFloETS.port, vol.heatPort) annotation (Line(points={{0,-54},{
          -20,-54},{-20,-10},{-9,-10}}, color={191,0,0}));
  connect(vol.ports[3], senTem.port)
    annotation (Line(points={{1,0},{0,0},{0,26}}, color={0,127,255}));
    annotation (defaultComponentName="heaOrCoo",
              Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end HeatingOrCooling;
