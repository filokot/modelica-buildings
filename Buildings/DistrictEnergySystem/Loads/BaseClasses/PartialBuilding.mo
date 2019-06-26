within Buildings.DistrictEnergySystem.Loads.BaseClasses;
partial model PartialBuilding "Partial class for building model"
  parameter Modelica.SIunits.HeatFlowRate Q_flowHea_nominal[nLoaHea]
    "Heating power at nominal conditions (always positive)"
    annotation(Dialog(group = "Nominal condition"));
  parameter Modelica.SIunits.HeatFlowRate Q_flowCoo_nominal[nLoaCoo]
    "Cooling power at nominal conditions (always positive)"
    annotation(Dialog(group = "Nominal condition"));

  parameter Integer nLoaHea = 1
    "Number of heating loads";

  parameter Integer nLoaCoo = 1
    "Number of cooling loads";

  parameter Buildings.DistrictEnergySystem.Loads.Types.ModelType heaLoaTyp[nLoaHea]=
    fill(Buildings.DistrictEnergySystem.Loads.Types.ModelType.HeatPort, nLoaHea)
    "Type of heating load model";

  parameter Buildings.DistrictEnergySystem.Loads.Types.ModelType cooLoaTyp[nLoaCoo]=
    fill(Buildings.DistrictEnergySystem.Loads.Types.ModelType.HeatPort, nLoaCoo)
    "Type of cooling load model";

  Buildings.BoundaryConditions.WeatherData.Bus weaBus "Weather data bus"
    annotation (Placement(
    transformation(extent={{-16,284},{18,316}}), iconTransformation(extent={{
            -16,84},{18,116}})));

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heaPorCoo[nLoaCoo]
    "Heat port for heat transfer with the cooling source"       annotation (
      Placement(transformation(extent={{-310,-110},{-290,-90}}),
        iconTransformation(extent={{-110,-80},{-90,-60}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heaPorHea[nLoaHea]
    "Heat port for heat transfer with the heating source"       annotation (
      Placement(transformation(extent={{-310,90},{-290,110}}),
        iconTransformation(extent={{-110,60},{-90,80}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput Q_flowCooReq[nLoaCoo](
    each quantity="HeatFlowRate", each unit="W")
    "Cooling heat flow rate required to meet setpoint"
    annotation (
      Placement(transformation(extent={{300,-110},{320,-90}}),
        iconTransformation(extent={{100,-40},{120,-20}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput Q_flowHeaReq[nLoaHea](
    each quantity="HeatFlowRate", each unit="W")
    "Heating heat flow rate required to meet setpoint"
    annotation (
      Placement(transformation(extent={{300,90},{320,110}}), iconTransformation(
          extent={{100,20},{120,40}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput Q_flowHeaAct[nLoaHea](
    each quantity="HeatFlowRate", each unit="W")
    "Actual heating heat flow rate"
    annotation (Placement(
    transformation(extent={{300,200},{320,220}}), iconTransformation(extent={{100,80},{120,100}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput Q_flowCooAct[nLoaCoo](
    each quantity="HeatFlowRate", each unit="W")
    "Actual cooling heat flow rate"
    annotation (Placement(
    transformation(extent={{300,-220},{320,-200}}), iconTransformation(extent={{100,-100},{120,-80}})));
  Modelica.Blocks.Sources.RealExpression Q_flowHea[nLoaHea](
    y={heaPorHea[i].Q_flow for i in 1:nLoaHea})
    annotation (Placement(transformation(extent={{260,200},{280,220}})));
  Modelica.Blocks.Sources.RealExpression Q_flowCoo[nLoaCoo](
    y={heaPorCoo[i].Q_flow for i in 1:nLoaCoo})
    annotation (Placement(transformation(extent={{260,-220},{280,-200}})));
  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant minTSet[nLoaHeaO](
    each k=20) if nLoaHeaO > 0
    "Minimum temperature setpoint"
    annotation (Placement(transformation(extent={{-260,140},{-240,160}})));
  Buildings.Controls.OBC.UnitConversions.From_degC from_degC[nLoaHeaO] if nLoaHeaO > 0
    annotation (Placement(transformation(extent={{-220,140},{-200,160}})));
  Buildings.DistrictEnergySystem.Loads.BaseClasses.SimplifiedBuildingODE buiODE[nLoaHeaO](
    final Q_flowHea_nominal={Q_flowHea_nominal[i] for i in 1:nLoaHeaO},
    each TOutHea_nominal=268.15,
    each TIndHea_nominal=293.15) if nLoaHeaO > 0 annotation (Placement(transformation(extent={{-200,90},{-220,110}})));
  Buildings.HeatTransfer.Sources.PrescribedTemperature TFromODE[nLoaHeaO] if nLoaHeaO > 0
    annotation (Placement(transformation(extent={{-240,90},{-260,110}})));
  Buildings.HeatTransfer.Sources.PrescribedTemperature prescribedT[nLoaHeaT] if nLoaHeaT > 0
    annotation (Placement(transformation(extent={{-240,40},{-260,60}})));
protected
  parameter Integer nLoaHeaH = Modelica.Math.BooleanVectors.countTrue(
    {el==Buildings.DistrictEnergySystem.Loads.Types.ModelType.HeatPort for el in heaLoaTyp});

  parameter Integer nLoaHeaT = Modelica.Math.BooleanVectors.countTrue(
    {el==Buildings.DistrictEnergySystem.Loads.Types.ModelType.PrescribedT for el in heaLoaTyp});

  parameter Integer nLoaHeaO = Modelica.Math.BooleanVectors.countTrue(
    {el==Buildings.DistrictEnergySystem.Loads.Types.ModelType.ODE for el in heaLoaTyp});
equation
  for i in 1:nLoaCoo loop
    connect(Q_flowCoo[i].y, Q_flowCooAct[i])
    annotation (Line(points={{281,-210},{292.5,-210},{292.5,-210},{310,-210}}, color={0,0,127}));
  end for;
  for i in 1:nLoaHea loop
    connect(Q_flowHea[i].y, Q_flowHeaAct[i])
    annotation (Line(points={{281,210},{310,210}}, color={0,0,127}));
  end for;

  // Loads at temperature computed by ODE model are the first to be connected.
  if nLoaHeaO > 0 then
    for i in 1:nLoaHeaO loop
      connect(minTSet[i].y, from_degC[i].u)
        annotation (Line(points={{-239,150},{-222,150}}, color={0,0,127}));
      connect(from_degC[i].y, buiODE[i].TSet)
        annotation (Line(points={{-199,150},{-180,150},{-180,102.8},{-198,102.8}}, color={0,0,127}));
      connect(weaBus.TDryBul, buiODE[i].TOut)
        annotation (Line(
          points={{1,300},{0.5,300},{0.5,108},{-198,108}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{6,3},{6,3}},
          horizontalAlignment=TextAlignment.Left));
      connect(Q_flowHea[i].y, buiODE[i].Q_flowAct)
        annotation (Line(points={{281,210},{288,210},{288,92},{-198,92}}, color={0,0,127}));
      connect(TFromODE[i].port, heaPorHea[i])
        annotation (Line(points={{-260,100},{-300,100}}, color={191,0,0}));
      connect(buiODE[i].TInd, TFromODE[i].T)
        annotation (Line(points={{-221,100},{-238,100}}, color={0,0,127}));
    end for;
  end if;
  // Loads at prescribed temperature are the next to be connected.
  if nLoaHeaT > 0 then
    for i in 1:nLoaHeaT loop
      connect(prescribedT[i + nLoaHeaO].port, heaPorHea[i + nLoaHeaO])
        annotation (Line(points={{-260,50},{-282,50},{-282,100},{-300,100}}, color={191,0,0}));
    end for;
  end if;
  // Loads represented by thermal model with heat port have to be connected by the user.


  annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
            {100,100}})),                                       Diagram(
        coordinateSystem(preserveAspectRatio=true, extent={{-300,-300},{300,300}})));
end PartialBuilding;
