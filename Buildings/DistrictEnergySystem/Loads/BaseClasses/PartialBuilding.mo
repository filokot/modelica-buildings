within Buildings.DistrictEnergySystem.Loads.BaseClasses;
partial model PartialBuilding "Partial class for building model"
  parameter Modelica.SIunits.HeatFlowRate Q_flowHea_nominal[nHeaLoa]
    "Heating power at nominal conditions (always positive)"
    annotation(Dialog(group = "Nominal condition"));
  parameter Modelica.SIunits.HeatFlowRate Q_flowCoo_nominal[nCooLoa]
    "Cooling power at nominal conditions (always positive)"
    annotation(Dialog(group = "Nominal condition"));

  parameter Integer nHeaLoa = 1
    "Number of heating loads";

  parameter Integer nCooLoa = 1
    "Number of cooling loads";

  parameter Buildings.DistrictEnergySystem.Loads.Types.ModelType heaLoaTyp[nHeaLoa]=
    fill(Buildings.DistrictEnergySystem.Loads.Types.ModelType.HeatPort, nHeaLoa)
    "Type of heating load model";

  parameter Buildings.DistrictEnergySystem.Loads.Types.ModelType cooLoaTyp[nCooLoa]=
    fill(Buildings.DistrictEnergySystem.Loads.Types.ModelType.HeatPort, nCooLoa)
    "Type of cooling load model";

  Buildings.BoundaryConditions.WeatherData.Bus weaBus "Weather data bus"
    annotation (Placement(
    transformation(extent={{-16,284},{18,316}}), iconTransformation(extent={{
            -16,84},{18,116}})));

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heaPorCoo[nCooLoa]
    "Heat port for heat transfer with the cooling source"       annotation (
      Placement(transformation(extent={{-310,-110},{-290,-90}}),
        iconTransformation(extent={{-110,-80},{-90,-60}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heaPorHea[nHeaLoa]
    "Heat port for heat transfer with the heating source"       annotation (
      Placement(transformation(extent={{-310,90},{-290,110}}),
        iconTransformation(extent={{-110,60},{-90,80}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput Q_flowCooReq[nCooLoa](
    each quantity="HeatFlowRate", each unit="W")
    "Cooling heat flow rate required to meet setpoint"
    annotation (
      Placement(transformation(extent={{300,-110},{320,-90}}),
        iconTransformation(extent={{100,-40},{120,-20}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput Q_flowHeaReq[nHeaLoa](
    each quantity="HeatFlowRate", each unit="W")
    "Heating heat flow rate required to meet setpoint"
    annotation (
      Placement(transformation(extent={{300,90},{320,110}}), iconTransformation(
          extent={{100,20},{120,40}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput Q_flowHeaAct[nHeaLoa](
    each quantity="HeatFlowRate", each unit="W")
    "Actual heating heat flow rate"
    annotation (Placement(
    transformation(extent={{300,200},{320,220}}), iconTransformation(extent={{100,80},{120,100}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput Q_flowCooAct[nCooLoa](
    each quantity="HeatFlowRate", each unit="W")
    "Actual cooling heat flow rate"
    annotation (Placement(
    transformation(extent={{300,-220},{320,-200}}), iconTransformation(extent={{100,-100},{120,-80}})));
  Modelica.Blocks.Sources.RealExpression Q_flowHea[nHeaLoa](
    y={heaPorHea[i].Q_flow for i in 1:nHeaLoa})
    annotation (Placement(transformation(extent={{260,200},{280,220}})));
  Modelica.Blocks.Sources.RealExpression Q_flowCoo[nCooLoa](
    y={heaPorCoo[i].Q_flow for i in 1:nCooLoa})
    annotation (Placement(transformation(extent={{260,-220},{280,-200}})));
  Buildings.DistrictEnergySystem.Loads.BaseClasses.SimplifiedBuildingODE heaLoaODE[nHeaLoaO](
    final Q_flowHea_nominal={Q_flowHea_nominal[i] for i in 1:nHeaLoaO},
    each TOutHea_nominal=268.15,
    each TIndHea_nominal=293.15) if nHeaLoaO > 0
    "ODE model computing the temperature of heating load"
    annotation (Placement(transformation(extent={{-200,90},{-220,110}})));
  Buildings.HeatTransfer.Sources.PrescribedTemperature THeaLoaODE[nHeaLoaO] if nHeaLoaO > 0
    "Temperature of heating load computed by ODE model"
    annotation (Placement(transformation(extent={{-240,90},{-260,110}})));
  HeatTransfer.Sources.PrescribedTemperature TCooLoaODE[nHeaLoaO] if nCooLoaO > 0
    "Temperature of cooling load computed by ODE model"
    annotation (Placement(transformation(extent={{-240,-110},{-260,-90}})));
  SimplifiedBuildingODE cooLoaODE[nCooLoaO](
    final Q_flowHea_nominal={Q_flowHea_nominal[i] for i in 1:nCooLoaO},
    final Q_flow_nominal={Q_flowCoo_nominal[i] for i in 1:nCooLoaO},
    each TOutHea_nominal=268.15,
    each TIndHea_nominal=293.15) if nCooLoaO > 0 "ODE model computing the temperature of cooling load"
    annotation (Placement(transformation(extent={{-200,-110},{-220,-90}})));
  HeatTransfer.Sources.PrescribedTemperature preTCooLoa[nCooLoaT] if nCooLoaT > 0
    "Prescribed temperature of cooling load"
    annotation (Placement(transformation(extent={{-240,-60},{-260,-40}})));
  Buildings.HeatTransfer.Sources.PrescribedTemperature preTHeaLoa[nHeaLoaT] if nHeaLoaT > 0
    "Prescribed temperature of heating load"
    annotation (Placement(transformation(extent={{-240,40},{-260,60}})));
protected
  parameter Integer nHeaLoaH = Modelica.Math.BooleanVectors.countTrue(
    {el==Buildings.DistrictEnergySystem.Loads.Types.ModelType.HeatPort for el in heaLoaTyp});
  parameter Integer heaLoaH_idx[nHeaLoaH] = Modelica.Math.BooleanVectors.index(
    {el==Buildings.DistrictEnergySystem.Loads.Types.ModelType.HeatPort for el in heaLoaTyp});
  parameter Integer nHeaLoaT = Modelica.Math.BooleanVectors.countTrue(
    {el==Buildings.DistrictEnergySystem.Loads.Types.ModelType.PrescribedT for el in heaLoaTyp});
  parameter Integer heaLoaT_idx[nHeaLoaT] = Modelica.Math.BooleanVectors.index(
    {el==Buildings.DistrictEnergySystem.Loads.Types.ModelType.PrescribedT for el in heaLoaTyp});
  parameter Integer nHeaLoaO = Modelica.Math.BooleanVectors.countTrue(
    {el==Buildings.DistrictEnergySystem.Loads.Types.ModelType.ODE for el in heaLoaTyp});
  parameter Integer heaLoaO_idx[nHeaLoaO] = Modelica.Math.BooleanVectors.index(
    {el==Buildings.DistrictEnergySystem.Loads.Types.ModelType.ODE for el in heaLoaTyp});
  parameter Integer nCooLoaH = Modelica.Math.BooleanVectors.countTrue(
    {el==Buildings.DistrictEnergySystem.Loads.Types.ModelType.HeatPort for el in cooLoaTyp});
  parameter Integer cooLoaH_idx[nCooLoaH] = Modelica.Math.BooleanVectors.index(
    {el==Buildings.DistrictEnergySystem.Loads.Types.ModelType.HeatPort for el in cooLoaTyp});
  parameter Integer nCooLoaT = Modelica.Math.BooleanVectors.countTrue(
    {el==Buildings.DistrictEnergySystem.Loads.Types.ModelType.PrescribedT for el in cooLoaTyp});
  parameter Integer cooLoaT_idx[nCooLoaT] = Modelica.Math.BooleanVectors.index(
    {el==Buildings.DistrictEnergySystem.Loads.Types.ModelType.PrescribedT for el in cooLoaTyp});
  parameter Integer nCooLoaO = Modelica.Math.BooleanVectors.countTrue(
    {el==Buildings.DistrictEnergySystem.Loads.Types.ModelType.ODE for el in cooLoaTyp});
  parameter Integer cooLoaO_idx[nCooLoaO] = Modelica.Math.BooleanVectors.index(
    {el==Buildings.DistrictEnergySystem.Loads.Types.ModelType.ODE for el in cooLoaTyp});
equation
  for i in 1:nCooLoa loop
    connect(Q_flowCoo[i].y, Q_flowCooAct[i])
    annotation (Line(points={{281,-210},{292.5,-210},{292.5,-210},{310,-210}}, color={0,0,127}));
  end for;
  for i in 1:nHeaLoa loop
    connect(Q_flowHea[i].y, Q_flowHeaAct[i])
    annotation (Line(points={{281,210},{310,210}}, color={0,0,127}));
  end for;
  if nHeaLoaO > 0 then
    for i in 1:nHeaLoaO loop
      connect(weaBus.TDryBul, heaLoaODE[i].TOut)
        annotation (Line(
          points={{1,300},{-159.5,300},{-159.5,108},{-198,108}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{6,3},{6,3}},
          horizontalAlignment=TextAlignment.Left));
      connect(heaLoaODE[i].TInd, THeaLoaODE[i].T)
        annotation (Line(points={{-221,100},{-238,100}}, color={0,0,127}));
      // Connection to I/O variables.
      connect(Q_flowHea[heaLoaO_idx[i]].y, heaLoaODE[i].Q_flowAct)
        annotation (Line(points={{281,210},{288,210},{288,92},{-198,92}}, color={0,0,127}));
      connect(THeaLoaODE[i].port, heaPorHea[heaLoaO_idx[i]])
        annotation (Line(points={{-260,100},{-300,100}}, color={191,0,0}));
    end for;
  end if;
  if nHeaLoaT > 0 then
    for i in 1:nHeaLoaT loop
      // Connection to I/O variables.
      connect(preTHeaLoa[i].port, heaPorHea[heaLoaT_idx[i]])
        annotation (Line(points={{-260,50},{-280,50},{-280,100},{-300,100}}, color={191,0,0}));
    end for;
  end if;
  if nCooLoaO > 0 then
    for i in 1:nCooLoaO loop
      connect(weaBus.TDryBul, cooLoaODE[i].TOut)
        annotation (Line(
        points={{1,300},{-160,300},{-160,-92},{-198,-92}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
      connect(cooLoaODE[i].TInd, TCooLoaODE[i].T)
        annotation (Line(points={{-221,-100},{-238,-100}}, color={0,0,127}));
      // Connection to I/O variables.
      connect(Q_flowCoo[cooLoaO_idx[i]].y, cooLoaODE[i].Q_flowAct)
        annotation (Line(points={{281,-210},{288,-210},{288,-108},{-198,-108}}, color={0,0,127}));
      connect(TCooLoaODE[i].port, heaPorCoo[cooLoaO_idx[i]])
        annotation (Line(points={{-260,-100},{-300,-100}}, color={191,0,0}));
    end for;
  end if;
  if nCooLoaT > 0 then
    for i in 1:nCooLoaT loop
      // Connection to I/O variables.
      connect(preTCooLoa[i].port, heaPorCoo[cooLoaT_idx[i]])
        annotation (Line(points={{-260,-50},{-280,-50},{-280,-100},{-300,-100}}, color={191,0,0}));
    end for;
  end if;

  annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
            {100,100}})),                                       Diagram(
        coordinateSystem(preserveAspectRatio=true, extent={{-300,-300},{300,300}})));
end PartialBuilding;
