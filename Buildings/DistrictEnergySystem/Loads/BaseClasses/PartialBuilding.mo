within Buildings.DistrictEnergySystem.Loads.BaseClasses;
partial model PartialBuilding "Partial class for building model"
  parameter Modelica.SIunits.HeatFlowRate Q_flowHea_nominal
    "Heating power at nominal conditions (always positive)"
    annotation(Dialog(group = "Nominal condition"));

  parameter Modelica.SIunits.HeatFlowRate Q_flowCoo_nominal
    "Cooling power at nominal conditions (always positive)"
    annotation(Dialog(group = "Nominal condition"));

  BoundaryConditions.WeatherData.Bus weaBus "Weather data bus"
    annotation (Placement(
    transformation(extent={{-16,284},{18,316}}), iconTransformation(extent={{
            -16,84},{18,116}})));

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heaPorCoo
    "Heat port for heat transfer with the cooling source"       annotation (
      Placement(transformation(extent={{-310,-110},{-290,-90}}),
        iconTransformation(extent={{-110,-80},{-90,-60}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heaPorHea
    "Heat port for heat transfer with the heating source"       annotation (
      Placement(transformation(extent={{-310,90},{-290,110}}),
        iconTransformation(extent={{-110,60},{-90,80}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yCoo(
    min=0, unit="1") "Cooling control signal"
                                   annotation (
      Placement(transformation(extent={{300,-110},{320,-90}}),
        iconTransformation(extent={{100,-40},{120,-20}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yHea(
    min=0, unit="1") "Heating control signal"
                                   annotation (
      Placement(transformation(extent={{300,90},{320,110}}), iconTransformation(
          extent={{100,20},{120,40}})));
  Controls.OBC.CDL.Interfaces.RealOutput dQ_flowHea(min=0, unit="W") "Unlet heating load" annotation (Placement(
        transformation(extent={{300,200},{320,220}}), iconTransformation(extent={{100,80},{120,100}})));
  Controls.OBC.CDL.Interfaces.RealOutput dQ_flowCoo(min=0, unit="W") "Unmet cooling load" annotation (Placement(
        transformation(extent={{300,-220},{320,-200}}), iconTransformation(extent={{100,-100},{120,-80}})));
  annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
            {100,100}})),                                       Diagram(
        coordinateSystem(preserveAspectRatio=true, extent={{-300,-300},{300,300}})));
end PartialBuilding;
