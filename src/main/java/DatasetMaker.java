/*
 * Copyright 2020 Eric Medvet <eric.medvet@gmail.com> (as eric)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.tasks.locomotion.Locomotion;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.RobotUtils;
import it.units.erallab.hmsrobots.util.SerializationUtils;
import it.units.erallab.hmsrobots.viewers.*;
import it.units.erallab.hmsrobots.viewers.drawers.Ground;
import it.units.erallab.hmsrobots.viewers.drawers.Lidar;
import it.units.erallab.hmsrobots.viewers.drawers.SensorReading;
import it.units.erallab.hmsrobots.viewers.drawers.Voxel;
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVRecord;
import org.apache.commons.lang3.tuple.Pair;
import org.dyn4j.dynamics.Settings;

import java.io.*;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.logging.Logger;

import static it.units.malelab.jgea.core.util.Args.*;

/**
 * @author eric
 */
public class DatasetMaker {

  private static final Logger L = Logger.getLogger(DatasetMaker.class.getName());
  private static final String dir = "/Users/federicopigozzi/Desktop/PhD/co-evolution_GECCO_21/VSRBiodiversity/";
  private static final String[] files = new String[] {/*"/Users/federicopigozzi/Desktop/PhD/co-evolution_GECCO_21/VSRBiodiversity/dataset_vlow.csv",*/
          "/Users/federicopigozzi/Desktop/PhD/co-evolution_GECCO_21/VSRBiodiversity/dataset_vhigh.csv"};

  public static void main(String[] args) throws IOException {
    String transformationName = a(args, "transformation", "identity");
    double startTime = d(a(args, "startTime", "5.0"));
    double endTime = d(a(args, "endTime", "25.0"));
    int w = i(a(args, "w", "200"));
    int h = i(a(args, "h", "130"));
    int frameRate = i(a(args, "frameRate", "30"));
    String encoderName = a(args, "encoder", VideoUtils.EncoderFacility.FFMPEG_LARGE.name());
    SerializationUtils.Mode mode = SerializationUtils.Mode.valueOf(a(args, "deserializationMode", SerializationUtils.Mode.GZIPPED_JSON.name()).toUpperCase());
    int count = 0;
    String terrainName;
    String outputFileName;
    for (String file : files) {
      for (CSVRecord record : CSVFormat.DEFAULT.withDelimiter(';').withFirstRecordAsHeader().parse(new FileReader(file)).getRecords()) {
        terrainName = record.get("terrain");
        outputFileName = dir + ((file.contains("low")) ? "dataset_vlow/" : "dataset_vhigh/") + String.join(".", terrainName, String.valueOf(count), "mp4");
        ++count;
        Grid<List<String>> rawGrid = Grid.create(
                1,
                1,
                (x, y) -> List.of(record.get("serialized")));
        //build named grid of robots
        Grid<Pair<String, Robot<?>>> namedRobotGrid = Grid.create(
                rawGrid.getW(),
                rawGrid.getH(),
                (x, y) -> rawGrid.get(x, y).isEmpty() ? null : Pair.of(
                        "" + " " + "",
                        RobotUtils.buildRobotTransformation(transformationName, new Random(0))
                                .apply(SerializationUtils.deserialize(rawGrid.get(x, y).get(0), Robot.class, mode))
                )
        );
        //prepare problem
        Locomotion locomotion = new Locomotion(
                endTime,
                Locomotion.createTerrain(terrainName),
                new Settings()
        );
        //do simulations
        ScheduledExecutorService uiExecutor = Executors.newScheduledThreadPool(4);
        ExecutorService executor = Executors.newCachedThreadPool();
        GridSnapshotListener gridSnapshotListener = null;
        try {
          gridSnapshotListener = new GridFileWriter(
                  w, h, startTime, frameRate, VideoUtils.EncoderFacility.valueOf(encoderName.toUpperCase()),
                  new File(outputFileName),
                  Grid.create(namedRobotGrid, p -> p == null ? null : p.getLeft()),
                  GraphicsDrawer.build().setConfigurable("drawers", List.of(
                          it.units.erallab.hmsrobots.viewers.drawers.Robot.build(),
                          Voxel.build(),
                          Ground.build(),
                          SensorReading.build(),
                          Lidar.build()
                  )).setConfigurable("generalRenderingModes", Set.of(
                          GraphicsDrawer.GeneralRenderingMode.TIME_INFO,
                          GraphicsDrawer.GeneralRenderingMode.VOXEL_COMPOUND_CENTERS_INFO
                  ))
          );
        } catch (IOException e) {
          L.severe(String.format("Cannot build grid file writer: %s", e));
          System.exit(-1);
        }
        GridEpisodeRunner<Robot<?>> runner = new GridEpisodeRunner<>(
                namedRobotGrid,
                locomotion,
                gridSnapshotListener,
                executor
        );
        runner.run();
        executor.shutdownNow();
        uiExecutor.shutdownNow();
      }
    }
  }

}