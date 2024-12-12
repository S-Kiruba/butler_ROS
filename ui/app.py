from flask import Flask, request, jsonify
from flask_cors import CORS
import rospy
from std_msgs.msg import String, Bool  # Import Bool instead of String

app = Flask(__name__)
CORS(app)  # Enable Cross-Origin Resource Sharing

# ROS setup
rospy.init_node('order_service', anonymous=True)
order_pub = rospy.Publisher('/new_order', String, queue_size=10)
cancel_pub = rospy.Publisher('/cancel_task', String, queue_size=10)
table_confirm_pub = rospy.Publisher('/table_confirm', Bool, queue_size=10)  # Changed to Bool
kitchen_confirm_pub = rospy.Publisher('/kitchen_confirm', Bool, queue_size=10)  # Changed to Bool

@app.route('/order', methods=['POST'])
def place_order():
    try:
        data = request.get_json()
        order = data.get('order')  # Expecting 'order' as "tableX"
        if order:
            #print(f"Publishing order to /new_order: {order}")
            order_pub.publish(order)  # Publish the order as "tableX"
            return jsonify({"message": f"Order for {order} placed successfully."}), 200
        return jsonify({"message": "Invalid order."}), 400
    except Exception as e:
        return jsonify({"message": str(e)}), 500

@app.route('/cancel', methods=['POST'])
def cancel_order():
    try:
        data = request.get_json()
        task = data.get('order')  # Expecting 'task' as "tableX"
        if task:
            #print(f"cancelled order to /cancel_order: {task}")
            cancel_pub.publish(task)  # Publish the cancel request as "tableX"
            return jsonify({"message": f"Order for {task} cancelled successfully."}), 200
        return jsonify({"message": "Invalid task."}), 400
    except Exception as e:
        return jsonify({"message": str(e)}), 500

@app.route('/table_confirm', methods=['POST'])
def table_confirm():
    try:
        data = request.get_json()
        confirmed = data.get('confirmed')  # Expecting 'confirmed' as True or False
        
        if isinstance(confirmed, bool):  # Validate that 'confirmed' is a boolean
            table_confirm_pub.publish(confirmed)  # Publish the boolean value
            return jsonify({"message": f"Confirmation status updated to {confirmed}."}), 200

        return jsonify({"message": "Invalid confirmation data. Expected a boolean value."}), 400
    except Exception as e:
        return jsonify({"message": str(e)}), 500
        
@app.route('/kitchen_confirm', methods=['POST'])
def kitchen_confirm():
    try:
        data = request.get_json()
        confirmed = data.get('confirmed')  # Expecting 'confirmed' as True or False
        
        if isinstance(confirmed, bool):  # Validate that 'confirmed' is a boolean
            kitchen_confirm_pub.publish(confirmed)  # Publish the boolean value
            return jsonify({"message": f"Kitchen confirmation status updated to {confirmed}."}), 200

        return jsonify({"message": "Invalid confirmation data. Expected a boolean value."}), 400
    except Exception as e:
        return jsonify({"message": str(e)}), 500

if __name__ == '__main__':
    app.run(debug=True)

