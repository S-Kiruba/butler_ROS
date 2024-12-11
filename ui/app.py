from flask import Flask, request, jsonify
from flask_cors import CORS
import rospy
from std_msgs.msg import String
import json

app = Flask(__name__)
CORS(app)  # Enable Cross-Origin Resource Sharing

# ROS setup
rospy.init_node('order_service', anonymous=True)
order_pub = rospy.Publisher('/new_order', String, queue_size=10)
cancel_pub = rospy.Publisher('/cancel_order', String, queue_size=10)
table_confirm_pub = rospy.Publisher('/table_confirm', String, queue_size=10)  # Topic for table confirmations
kitchen_confirm_pub = rospy.Publisher('/kitchen_confirm', String, queue_size=10)  # Topic for kitchen confirmations

@app.route('/order', methods=['POST'])
def place_order():
    try:
        data = request.get_json()
        order = data.get('order')
        if order:
            order_pub.publish(order.lower())  # Publish the order (convert to lowercase)
            return jsonify({"message": f"Order for {order} placed successfully."}), 200
        return jsonify({"message": "Invalid order."}), 400
    except Exception as e:
        return jsonify({"message": str(e)}), 500

@app.route('/cancel', methods=['POST'])
def cancel_order():
    try:
        data = request.get_json()
        task = data.get('task')
        if task:
            cancel_pub.publish(task.lower())  # Publish the cancel request (convert to lowercase)
            return jsonify({"message": f"Order for {task} cancelled successfully."}), 200
        return jsonify({"message": "Invalid task."}), 400
    except Exception as e:
        return jsonify({"message": str(e)}), 500

@app.route('/table_confirm', methods=['POST'])
def table_confirm():
    try:
        data = request.get_json()
        confirmed = data  # Expecting only a boolean (true/false) in the request body

        if isinstance(confirmed, bool):
            # Convert the boolean to a string before publishing
            table_confirm_pub.publish(str(confirmed).lower())  # Publish 'true' or 'false' as a string
            return jsonify({"message": f"Confirmation status updated to {confirmed}."}), 200

        return jsonify({"message": "Invalid confirmation data. Expected a boolean value."}), 400
    except Exception as e:
        return jsonify({"message": str(e)}), 500
        
@app.route('/kitchen_confirm', methods=['POST'])
def kitchen_confirm():
    try:
        data = request.get_json()
        confirmed = data  # Expecting only a boolean (true/false) in the request body

        if isinstance(confirmed, bool):
            # Publish the kitchen confirmation status (true/false) to the ROS topic
            kitchen_confirm_pub.publish(str(confirmed).lower())  # Directly publish the boolean value
            return jsonify({"message": f"Kitchen confirmation status updated to {confirmed}."}), 200

        return jsonify({"message": "Invalid confirmation data. Expected a boolean value."}), 400
    except Exception as e:
        return jsonify({"message": str(e)}), 500

if __name__ == '__main__':
    app.run(debug=True)

