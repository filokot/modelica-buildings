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
    min=0, max=1) "Control signal for cooling"
                                   annotation (
      Placement(transformation(extent={{300,-110},{320,-90}}),
        iconTransformation(extent={{100,-80},{120,-60}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yHea(
    min=0, max=1) "Control signal for heating"
                                   annotation (
      Placement(transformation(extent={{300,90},{320,110}}), iconTransformation(
          extent={{100,60},{120,80}})));
  annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
            {100,100}})),                                       Diagram(
        coordinateSystem(preserveAspectRatio=true, extent={{-300,-300},{300,300}})));
end PartialBuilding;
