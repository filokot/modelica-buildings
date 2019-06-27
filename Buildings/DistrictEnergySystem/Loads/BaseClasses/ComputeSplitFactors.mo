within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model ComputeSplitFactors
  extends Modelica.Blocks.Icons.Block;
  parameter Integer nLoa = 1
    "Number of connected loads";
  parameter Real Q_flow_small = 1;
  Buildings.Controls.OBC.CDL.Interfaces.RealInput Q_flow[nLoa](
    quantity="HeatFlowRate")
    "Heat flow rate" annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,0}),  iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,0})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput splFac[nLoa]
    "Vector of split factors"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
algorithm
  splFac := Buildings.Utilities.Math.Functions.regStep(
     sum(abs(Q_flow)),
     abs(Q_flow) / sum(abs(Q_flow)),
     fill(1/nLoa, nLoa),
     Q_flow_small);
  splFac := splFac / sum(splFac);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end ComputeSplitFactors;
