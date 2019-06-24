within Buildings.DistrictEnergySystem.Loads.BaseClasses;
partial model PartialBuildingNoPort "Partial class for building model"
  parameter Modelica.SIunits.HeatFlowRate Q_flowHea_nominal
    "Heating power at nominal conditions (always positive)"
    annotation(Dialog(group = "Nominal condition"));

  parameter Modelica.SIunits.HeatFlowRate Q_flowCoo_nominal
    "Cooling power at nominal conditions (always positive)"
    annotation(Dialog(group = "Nominal condition"));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput Q_flowHeaReq(
    quantity="HeatFlowRate",
    unit="W",
    displayUnit="W") "Heating power required" annotation (Placement(
        transformation(extent={{300,210},{320,230}}), iconTransformation(extent=
           {{100,30},{120,50}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput Q_flowCooReq(
    quantity="HeatFlowRate",
    unit="W",
    displayUnit="W") "Cooling power required" annotation (Placement(
        transformation(extent={{300,-232},{320,-212}}), iconTransformation(
          extent={{100,-50},{120,-30}})));

  BoundaryConditions.WeatherData.Bus weaBus "Weather data bus"
    annotation (Placement(
    transformation(extent={{-16,284},{18,316}}), iconTransformation(extent={{
            -16,84},{18,116}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput Q_flowHeaAva(
    quantity="HeatFlowRate",
    unit="W",
    displayUnit="W") "Heating power available"
                     annotation (Placement(transformation(extent={{-340,80},{
            -300,120}}),
                   iconTransformation(extent={{-140,40},{-100,80}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput Q_flowCooAva(
    quantity="HeatFlowRate",
    unit="W",
    displayUnit="W") "Cooling power available"
                     annotation (Placement(transformation(extent={{-340,-120},{
            -300,-80}}), iconTransformation(extent={{-140,-80},{-100,-40}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput TInd(
    unit="K",
    displayUnit="degC",
    quantity="ThermodynamicTemperature")
                        annotation (Placement(transformation(extent={{300,-10},
            {320,10}}), iconTransformation(extent={{100,-10},{120,10}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput Q_flowHeaAct(
    quantity="HeatFlowRate",
    unit="W",
    displayUnit="W") "Actual heating power" annotation (Placement(
        transformation(extent={{300,260},{320,280}}), iconTransformation(extent=
           {{100,70},{120,90}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput Q_flowCooAct(
    quantity="HeatFlowRate",
    unit="W",
    displayUnit="W") "Actual cooling power" annotation (Placement(
        transformation(extent={{300,-280},{320,-260}}), iconTransformation(
          extent={{100,-90},{120,-70}})));
  Buildings.Utilities.Math.SmoothMin                  smoothMin(deltaX=1E-2*
        Q_flowHea_nominal)
    "Limiting heat flow rate to currently available heating power"
    annotation (Placement(transformation(extent={{220,260},{240,280}})));
  Buildings.Utilities.Math.SmoothMax           smoothMax(deltaX=1E-2*
        Q_flowCoo_nominal)
    "Limiting heat flow rate to currently available cooling power"
    annotation (Placement(transformation(extent={{220,-280},{240,-260}})));
equation
  connect(smoothMin.y, Q_flowHeaAct)
    annotation (Line(points={{241,270},{310,270}},
                                                color={0,0,127}));
  connect(smoothMin.u2, Q_flowHeaReq) annotation (Line(points={{218,264},{200,
          264},{200,220},{310,220}},
                            color={0,0,127}));
  connect(Q_flowHeaAva, smoothMin.u1) annotation (Line(points={{-320,100},{-280,
          100},{-280,276},{218,276}},
                             color={0,0,127}));
  connect(smoothMax.y, Q_flowCooAct)
    annotation (Line(points={{241,-270},{310,-270}},
                                                  color={0,0,127}));
  connect(Q_flowCooAva, smoothMax.u2) annotation (Line(points={{-320,-100},{
          -280,-100},{-280,-276},{218,-276}},
                               color={0,0,127}));
  connect(smoothMax.u1, Q_flowCooReq) annotation (Line(points={{218,-264},{200,
          -264},{200,-222},{310,-222}},
                               color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
            {100,100}})),                                       Diagram(
        coordinateSystem(preserveAspectRatio=true, extent={{-300,-300},{300,300}})));
end PartialBuildingNoPort;
