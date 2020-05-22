from rosgraph_monitor.srv import PredictAction, PredictActionResponse
from std_msgs.msg import String
import roslibpy
from sklearn.metrics import mean_absolute_error
from sklearn.tree import DecisionTreeRegressor
from sklearn.model_selection import train_test_split
from sklearn import metrics
import seaborn as sns
import pandas as pd
pd.plotting.register_matplotlib_converters


class NavModel:
    def __init__(self, file_name, num_sens, model_name=""):
        self._data_set = pd.read_csv(file_name, header=None)
        self._num_sens = num_sens
        self._nav_model = DecisionTreeRegressor()

        # the code needs python3 and melodic is on python2
        # so interface the module with ROS
        self.ros_client = roslibpy.Ros('localhost', 9090)
        self.ros_client.on_ready(lambda: print(
            'Is ROS connected?', self.ros_client.is_connected))
        service = roslibpy.Service(
            self.ros_client, '/turtle_action', 'rosgraph_monitor/PredictAction')
        service.advertise(self._handle_sensor_data)

    def _handle_sensor_data(self, req, resp):
        if len(req['sensor_data']) == self._num_sens:
            action = self.get_action(req['sensor_data'])
            resp['action'] = action
            return True
        return False

    def prepare_data(self):
        # adding header row to the raw dataframe
        print("reading and processing data-sets")

        self._sensor_labels = list()
        self._action_labels = list()
        prefix = 'sensor_'
        for i in range(self._num_sens):
            name = prefix + str(i+1)
            self._sensor_labels.append(name)
        self._action_labels.append("action")
        self._data_set.columns = self._sensor_labels + self._action_labels

        # Converting actions from string to int
        classes = ("Move-Forward", "Slight-Right-Turn",
                   "Sharp-Right-Turn", "Slight-Left-Turn")
        for i, item in enumerate(classes):
            self._data_set = self._data_set.replace(to_replace=item, value=i)
        # return self._data_set

    def train(self):
        # Trainng the model using the training data
        print("training model started...")
        x = self._data_set[self._sensor_labels]
        y = self._data_set[self._action_labels]
        train_x, test_X, train_y, test_y = train_test_split(
            x, y, random_state=0)

        self._nav_model.fit(train_x, train_y)
        action = self._predict_action(test_X, test_y)
        self._eval_metrics(test_y, action)

    def _predict_action(self, test_data, output_data):
        # The output prediction using the model generated happens here
        predicted_action = self._nav_model.predict(test_data)
        print("\nPredicted action = ", predicted_action)
        return predicted_action

    def _decode_action(self, action):
        # Decodes the predicted output to readable format
        return {
            0: "Move-Forward",
            1: "Slight-Right-Turn",
            2: "Sharp-Right-Turn",
            3: "Slight-Left-Turn"
        }[action]

    def _eval_metrics(self, output_data, predicted_action):
        print("\t\t\tEvaluation Metrics")
        mae = mean_absolute_error(output_data, predicted_action)
        acc = metrics.accuracy_score(output_data, predicted_action)
        print("Mean Absolute Error:\t", mae)
        print("Accuracy of model:\t", acc*100, "%")

    def get_action(self, sensor_raw):
        # This is where you need to feed the data from turtle bot
        sensor_input = pd.DataFrame(sensor_raw).T
        prediction = self._nav_model.predict(sensor_input)
        action = self._decode_action(prediction[0])
        return action

    def start_service(self):
        self.ros_client.run_forever()


def main():
    model = NavModel(
        'resources/sensor_readings_24.csv', 24)
    model.prepare_data()
    model.train()  # should take the sensor array
    model.start_service()

    sensor_raw = [0.382, 0.612, 0.584, 3.665, 2.953, 2.940, 2.740, 2.629, 1.709, 2.311, 1.860,
                  1.640, 1.635, 1.654, 1.755, 0.263, 0.545, 0.475, 0.475, 0.185, 0.464, 0.259, 0.468, 0.278]
    action = model.get_action(sensor_raw)
    print("\nPrediction by the model:\t", action)


if __name__ == "__main__":
    main()
